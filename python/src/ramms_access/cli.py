"""ramms-access CLI: run a device adapter against a RAMMS sim or robot bridge."""

from __future__ import annotations

import argparse

from ramms_access.transport import DEFAULT_PORT, UdpIntentPublisher


def main() -> None:
    parser = argparse.ArgumentParser(prog="ramms-access",
                                     description="Publish alternative-access intents to RAMMS")
    parser.add_argument("--host", default="127.0.0.1", help="consumer host")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="consumer UDP port")
    parser.add_argument("--rate", type=float, default=50.0, help="publish rate (Hz)")
    sub = parser.add_subparsers(dest="adapter", required=True)

    p_demo = sub.add_parser("demo", help="synthetic patterns (no hardware)")
    p_demo.add_argument("--pattern", default="circle",
                        choices=["circle", "weave", "arm-wave", "idle"])

    sub.add_parser("pointer", help="OS pointer as virtual joystick (Neuralink/head-mouse/eye-gaze path)")

    p_galea = sub.add_parser("galea", help="OpenBCI Galea via BrainFlow (EMG v0)")
    p_galea.add_argument("--ip", default="", help="Galea IP (default: autodiscover)")
    p_galea.add_argument("--left-ch", type=int, default=0)
    p_galea.add_argument("--right-ch", type=int, default=1)
    p_galea.add_argument("--calibrate", action="store_true",
                         help="record relax/clench thresholds and exit")

    p_g0 = sub.add_parser("galea-p0", help="Galea phase 0: live monitor of every channel; sends nothing")
    p_g0.add_argument("--ip", default="", help="Galea IP (default: autodiscover)")

    p_g1 = sub.add_parser("galea-p1",
                          help="Galea phase 1: EMG gestures drive the chair "
                               "(clench=forward, smirk=turn, relax=stop)")
    p_g1.add_argument("--ip", default="", help="Galea IP (default: autodiscover)")
    p_g1.add_argument("--calibrate", action="store_true",
                      help="record the four gestures, save the gesture model, exit")

    p_g2 = sub.add_parser("galea-p2.0",
                          help="Galea + Neon phase 2.0: clench cycles REST -> ROTATE (gaze steers) "
                               "-> DRIVE (straight ahead)")
    p_g2.add_argument("--ip", default="", help="Galea IP (default: autodiscover)")
    p_g2.add_argument("--neon-ip", default="", help="Neon Companion phone IP (required unless --calibrate)")
    p_g2.add_argument("--calibrate", action="store_true",
                      help="record rest and clench, save the clench model, exit (Galea only)")

    sub.add_parser("chair-p0", help="sim chair phase 0: live heading readout via Remote Execution; sends nothing")

    p_n0 = sub.add_parser("neon-p0", help="Neon phase 0: live gaze / IMU / eye-event monitor; sends nothing")
    p_n0.add_argument("--ip", required=True, help="Companion phone IP (shown in the Neon Companion app)")

    args = parser.parse_args()
    publisher = UdpIntentPublisher(args.host, args.port)

    if args.adapter == "demo":
        from ramms_access.adapters.demo import DemoAdapter
        adapter = DemoAdapter(publisher, args.rate, args.pattern)
    elif args.adapter == "pointer":
        from ramms_access.adapters.pointer_hid import PointerAdapter
        adapter = PointerAdapter(publisher, args.rate)
    elif args.adapter == "galea":
        from ramms_access.adapters.galea import GaleaAdapter
        adapter = GaleaAdapter(publisher, args.rate, args.ip, args.left_ch, args.right_ch)
        if args.calibrate:
            adapter.calibrate()
            return
    elif args.adapter == "galea-p0":
        from ramms_access.adapters.galea_phase_0 import monitor
        monitor(args.ip)
        return
    elif args.adapter == "galea-p1":
        from ramms_access.adapters.galea_phase_1 import GaleaPhase1Adapter
        adapter = GaleaPhase1Adapter(publisher, args.rate, args.ip)
        if args.calibrate:
            adapter.calibrate()
            return
    elif args.adapter == "galea-p2.0":
        from ramms_access.adapters.galea_phase_2_0 import GaleaPhase2Adapter
        adapter = GaleaPhase2Adapter(publisher, args.rate, args.ip, args.neon_ip)
        if args.calibrate:
            adapter.calibrate()
            return
        if not args.neon_ip:
            p_g2.error("--neon-ip is required (except with --calibrate)")
    elif args.adapter == "chair-p0":
        from ramms_access.adapters.chair_phase_0 import monitor
        monitor()
        return
    elif args.adapter == "neon-p0":
        from ramms_access.adapters.neon_phase_0 import monitor
        monitor(args.ip)
        return
    else:  # pragma: no cover
        raise SystemExit(f"unknown adapter {args.adapter}")

    print(f"[ramms-access] {args.adapter} -> udp://{args.host}:{args.port} @ {args.rate}Hz "
          f"(Ctrl-C sends estop)")
    adapter.run()


if __name__ == "__main__":
    main()
