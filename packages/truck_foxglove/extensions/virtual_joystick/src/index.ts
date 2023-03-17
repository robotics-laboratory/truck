import { ExtensionContext } from "@foxglove/studio";
import { initTruckControlPanel } from "./truckControlPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "truck control", initPanel: initTruckControlPanel });
}
