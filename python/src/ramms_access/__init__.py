"""ramms-access: alternative-access input hub for RAMMS.

Device adapters (BCI, HID/pointer, AT switches) normalize their input into a
versioned intent stream (UDP JSON v1, see doc/PLAN.md) consumed by the RAMMS
simulator's URammsAccessInputComponent — and, later, by the physical robot's
bridge. The intent schema is the contract; devices and decoders iterate
freely behind it.
"""

from ramms_access.intents import Intent
from ramms_access.transport import UdpIntentPublisher

__all__ = ["Intent", "UdpIntentPublisher"]
__version__ = "0.1.0"
