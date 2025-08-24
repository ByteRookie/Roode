from pathlib import Path
import shutil

# Use device name if provided, default to roode32
node = data.get("device", "roode32")
config_dir = Path(hass.config.path())

# Find newest calibration directory
sessions = sorted(config_dir.glob("calibration_*"))
if not sessions:
    logger.warning("No calibration sessions found")
else:
    latest = sessions[-1]
    src = latest / "roi_result.json"
    if not src.exists():
        logger.warning("No roi_result.json in %s", latest)
    else:
        dst = config_dir / "roi_result.json"
        shutil.copy(src, dst)
        logger.info("Copied %s to %s for %s", src, dst, node)
