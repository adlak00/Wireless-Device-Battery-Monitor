// Logitech G435 LIGHTSPEED headset on the older 046D:0ACB dongle (HDT647, Realtek ALC4021 USB
// bridge in front of an Airoha/Audiowise radio SoC). Ported from deddroid/logitech-g435-battery
// (MIT License, Copyright (c) 2026 deddroid; full text in installer/THIRD_PARTY_NOTICES.txt;
// src/device.rs + docs/PROTOCOL.md), whose protocol was measured on real hardware and builds
// on endes' research (dev.to "USB HID down the rabbit hole: Logitech G435 dongle").
//
// Transport: vendor collection FFC0, report 6.
//   * Bridge messages go out as the 514-byte OUTPUT report (515 with the report ID):
//       06 01 03                       enter the bridge "bootloader" (ACK 06 5A, or 06 FE if already in)
//       06 04 03 <len lo> <len hi> <PA>  pass a "PA" command to the SoC  (ACK 06 5A)
//       06 08 03                       leave the bridge "bootloader"    (ACK 06 5A)
//     ACKs arrive on the interrupt IN endpoint (7-byte input report 6).
//   * SoC replies ("PI") are polled with GET_FEATURE on report 6 (63 bytes + ID); an empty buffer
//     reads 06 FF FF ...
//
// The "bootloader" is only a routing mode of the USB bridge; nothing is ever written to flash. Only
// three read commands are sent: REMOTE_GET_CONNECT_STATUS, GET_BATTERY and REMOTE_SET_MODE 1 (a
// runtime routing switch that does not persist). OTA / flash / NVDS commands are never sent.
//
// Caveats from the measurements:
//   * While the bridge is in that mode the dongle stops audio and microphone; with this firmware a
//     read costs about 0.6 s of silence. So the battery is read at most once an hour (every
//     2 minutes while the headset is off), and the last value is shown in between. The tray menu can
//     pause reads entirely (G435AcbSetPaused) when an uninterrupted connection matters.
//   * GET_BATTERY must never be the first command of a session: sent first it has left the dongle
//     answering nothing until replugged. The session always starts with GET_CONNECT_STATUS.
//   * The session always leaves the bootloader, also on error.
#include "drivers.h"
#include "logitech_g435_0adf.h"  // shared voltage -> percent curve (G435Adf::Percent)

namespace {

constexpr USHORT kVid = 0x046D;
constexpr USHORT kPid = 0x0ACB;
constexpr BYTE kReportId = 6;
constexpr BYTE kAckOk = 0x5A;
constexpr BYTE kAckAlready = 0xFE;

constexpr BYTE kCatOta = 0x06;
constexpr BYTE kCatWireless = 0x0E;
constexpr BYTE kRemoteSetMode = 0xE1;
constexpr BYTE kRemoteGetConnectStatus = 0xE3;

constexpr ULONGLONG kReadEveryMs = 60ull * 60 * 1000;  // headset on: one audio drop per hour
constexpr ULONGLONG kRetryOffMs = 2ull * 60 * 1000;    // headset off / no answer

// ---- pure parsing (from deddroid's protocol.rs) -------------------------------------------------

// GET_BATTERY reply: 07 00 06 <seq|80> 00 <mV lo> <mV hi>. Returns the raw value or -1.
int ParseBatteryRaw(const std::vector<BYTE>& b) {
    for (size_t i = 0; i + 7 <= b.size(); i++)
        if (b[i] == 0x07 && b[i + 1] == 0x00 && b[i + 2] == kCatOta && (b[i + 3] & 0x80) && b[i + 4] == 0x00)
            return b[i + 5] | (b[i + 6] << 8);
    return -1;
}

// In remote mode 0 the dongle answers GET_BATTERY itself with garbage (105, 210 seen).
bool IsBatteryMv(int mv) { return mv >= 2500 && mv <= 4600; }

// Wireless-category "get" reply: 02 00 <cmd> <value>. Returns the value or -1.
int ParseRemoteValue(const std::vector<BYTE>& b, BYTE cmd) {
    for (size_t i = 0; i + 4 <= b.size(); i++)
        if (b[i] == 0x02 && b[i + 1] == 0x00 && b[i + 2] == cmd) return b[i + 3];
    return -1;
}

// ---- bridge I/O ---------------------------------------------------------------------------------

// Sends a bridge message as the output report and waits for its ACK on the input endpoint,
// skipping interleaved volume-key reports (report 3). Returns the ACK byte or -1.
int Bridge(HidDev& d, BYTE op0, BYTE op1, const std::vector<BYTE>& body = {}) {
    std::vector<BYTE> report = { kReportId, op0, op1 };
    report.insert(report.end(), body.begin(), body.end());
    if (!d.Write(report)) return -1;  // HidDev pads to the 515-byte output report length
    ULONGLONG end = GetTickCount64() + 800;
    std::vector<BYTE> r;
    while (GetTickCount64() < end) {
        if (d.Read(r, 100) >= 2 && r[0] == kReportId) return r[1];
    }
    return -1;
}

void Drain(HidDev& d) {
    std::vector<BYTE> b;
    for (int i = 0; i < 16; i++)
        if (!d.GetFeature(kReportId, b) || b.size() < 2 || b[1] == 0xFF) break;
}

struct Session {
    HidDev& d;
    BYTE seq = 1;
    ~Session() { Bridge(d, 0x08, 0x03); }  // always leave the bootloader

    // Sends a "PA" command and polls feature reports (up to 1.5 s) until `parse` returns >= 0.
    template <class Parse>
    int Query(BYTE category, BYTE len, std::initializer_list<BYTE> payload, Parse parse) {
        std::vector<BYTE> pa = { 0x50, 0x41, category, seq, len };
        pa.insert(pa.end(), payload);
        seq = (BYTE)(seq % 127 + 1);
        std::vector<BYTE> msg = { (BYTE)(pa.size() & 0xFF), (BYTE)(pa.size() >> 8) };
        msg.insert(msg.end(), pa.begin(), pa.end());
        if (Bridge(d, 0x04, 0x03, msg) < 0) return -1;

        ULONGLONG end = GetTickCount64() + 1500;
        std::vector<BYTE> b;
        while (GetTickCount64() < end) {
            if (!d.GetFeature(kReportId, b)) return -1;
            if (b.size() > 1 && b[1] != 0xFF) {
                int v = parse(b);
                if (v >= 0) return v;
                continue;  // another reply queued before ours
            }
            Sleep(30);
        }
        return -1;
    }
    int BatteryRaw() { return Query(kCatOta, 1, { 0x00, 0x31 }, ParseBatteryRaw); }
};

enum class Result { Ok, HeadsetOff, NoReply };

Result ReadMillivolts(HidDev& d, int& mv) {
    int ack = Bridge(d, 0x01, 0x03);
    if (ack < 0) return Result::NoReply;
    (void)kAckOk; (void)kAckAlready;  // any ACK is fine: 5A entered, FE already in
    Session s{ d };
    Drain(d);

    // First command is never GET_BATTERY (see the header).
    int connected = s.Query(kCatWireless, 1, { kRemoteGetConnectStatus },
                            [](const std::vector<BYTE>& b) { return ParseRemoteValue(b, kRemoteGetConnectStatus); });
    if (connected == 0) return Result::HeadsetOff;
    if (connected < 0) return Result::NoReply;  // no sane reply: do not push GET_BATTERY at it

    mv = s.BatteryRaw();
    if (IsBatteryMv(mv)) return Result::Ok;

    // Remote mode 1 routes OTA commands to the headset (dongle starts in mode 0 and falls back at times).
    s.Query(kCatWireless, 2, { kRemoteSetMode, 0x01 }, [](const std::vector<BYTE>&) { return 0; });
    mv = s.BatteryRaw();
    return IsBatteryMv(mv) ? Result::Ok : Result::NoReply;
}

// Last result, so the audio is interrupted only every kReadEveryMs.
struct Cache {
    bool have = false;
    ULONGLONG next = 0;
    DeviceStatus st;
};
Cache g_cache;
SRWLOCK g_lock = SRWLOCK_INIT;
volatile LONG g_paused = 0;  // tray menu: never talk to the dongle
volatile LONG g_force = 0;   // tray menu "Refresh now": read on the next poll

}  // namespace

void PollLogitechG435Acb(PollContext& ctx) {
    if (ctx.IsClaimed(kVid, kPid)) return;
    auto colls = ctx.Find(kVid, kPid, [](const HidColl& c) {
        return c.page() == 0xFFC0 && c.outLen() == 515 && c.featLen() == 64 && c.inLen() >= 2;
    });
    AcquireSRWLockExclusive(&g_lock);
    if (colls.empty()) {
        g_cache = {};  // dongle unplugged: read again as soon as it is back
        ReleaseSRWLockExclusive(&g_lock);
        return;
    }
    ULONGLONG now = GetTickCount64();
    bool paused = InterlockedCompareExchange(&g_paused, 0, 0) != 0;
    bool force = !paused && InterlockedExchange(&g_force, 0) != 0;
    if (paused || (g_cache.have && now < g_cache.next && !force)) {
        DeviceStatus s;
        if (g_cache.have) {
            s = g_cache.st;
        } else {  // paused before the first read: list the headset without a level
            s.key = L"g435:" + Hex4(kPid);
            s.name = L"Logitech G435";
            s.kind = DeviceKind::Headset;
            s.via = L"2.4 GHz receiver";
        }
        if (paused) s.detail = L"odczyty baterii wstrzymane";
        ReleaseSRWLockExclusive(&g_lock);
        ctx.Add(s, kVid, kPid);
        return;
    }

    DeviceStatus s;
    s.key = L"g435:" + Hex4(kPid);
    s.name = L"Logitech G435";
    s.kind = DeviceKind::Headset;
    s.via = L"2.4 GHz receiver";

    HidDev d;
    int mv = -1;
    Result r = d.Open(*colls[0]) ? ReadMillivolts(d, mv) : Result::NoReply;
    if (r == Result::Ok) {
        s.millivolts = mv;
        s.percent = G435Adf::Percent(mv);
        g_cache.next = now + kReadEveryMs;
    } else if (r == Result::NoReply && g_cache.have && g_cache.st.connected) {
        s = g_cache.st;  // a missed read: keep the last value, try again sooner
        g_cache.next = now + kRetryOffMs;
    } else {
        s.connected = false;  // headset off or out of range
        g_cache.next = now + kRetryOffMs;
    }
    g_cache.have = true;
    g_cache.st = s;
    ReleaseSRWLockExclusive(&g_lock);
    ctx.Add(s, kVid, kPid);
}

void G435AcbSetPaused(bool on) { InterlockedExchange(&g_paused, on ? 1 : 0); }
bool G435AcbPaused() { return InterlockedCompareExchange(&g_paused, 0, 0) != 0; }
void G435AcbForceRead() { InterlockedExchange(&g_force, 1); }
