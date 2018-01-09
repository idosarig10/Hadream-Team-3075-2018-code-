package LibPurple.watchdog;

import LibPurple.utils.Utils;

public class BatteryWD implements Watchdog
{
	@Override
	public void execute() {
		Utils.batteryWatcher();
	}

}
