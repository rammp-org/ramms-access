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
    else:  # pragma: no cover
        raise SystemExit(f"unknown adapter {args.adapter}")

    print(f"[ramms-access] {args.adapter} -> udp://{args.host}:{args.port} @ {args.rate}Hz "
          f"(Ctrl-C sends estop)")
    adapter.run()


if __name__ == "__main__":
    main()
