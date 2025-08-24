from datetime import datetime
import json
from pathlib import Path

# Determine the device name from data or fall back to roode32
node = data.get("device", "roode32")

# Create session directory using timestamp
session_name = datetime.now().strftime("passive_scan_%Y-%m-%d_%H-%M")
session_dir = Path(hass.config.path(session_name))
session_dir.mkdir(parents=True, exist_ok=True)

# Initialize session.json as an empty list
(session_dir / "session.json").write_text(json.dumps([]))

# Dispatch start command to ESPHome node
service = f"{node}_start_passive_scan"
hass.services.call("esphome", service, {})
