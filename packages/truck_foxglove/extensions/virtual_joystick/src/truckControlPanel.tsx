import { PanelExtensionContext, RenderState, SettingsTree, SettingsTreeAction } from "@foxglove/studio";
import { useLayoutEffect, useEffect, useState, useRef } from "react";
import ReactDOM from "react-dom";
import {create, JoystickManager, JoystickManagerOptions, Position} from 'nipplejs';
import Button from '@mui/material/Button';
import { red } from '@mui/material/colors';

const primary = red[500];

type JoyMessage = {
  header: 
  {
    frame_id: string,
    stamp: {
      sec: number,
      nsec: number,
    },
  },
  axes: Array<number>,
  buttons: Array<number>,
};

const controlMap = {
  axes: 
  {
    y: 1,
    x: 0,
  },
  buttons:
  {
    off: 0,
    remote: 1,
    auto: 2,
  },
};

const publishRate = 300;

function createMsg(frame_id: string, curvature: number, velocity: number): JoyMessage {
  let nigger = new Date();
  return { 
    header: { 
      frame_id: frame_id, 
      stamp: { 
        sec: Math.floor(nigger.getTime() / 1000),
        nsec: nigger.getMilliseconds() * 1e+6,
      },
    }, 
    axes: [curvature, velocity], 
    buttons: [0, 0, 0] };
};

function TruckControlPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [currentMode, setCurrentMode] = useState<string>("Off");
  const [settingsFrameId, setFrameId] = useState<string>("base_link");
  const [settingsTopic, setTopic] = useState<string>("/truck/joy");

  const frame_id = useRef<string>("base_link");
  const currentTopic = useRef<string>("/truck/joy");
  const latestMsg = useRef<JoyMessage>(createMsg(frame_id.current, 0, 0));

  let startPos: Position;

  let truck_stick: JoystickManager;

  let createJoystick = ((size?: number) => {
    truck_stick = create({
      zone: document.getElementById("truck_stick") as HTMLDivElement,
      color: 'white',
      size: size ? size : 200,
      //size: Math.floor(Math.min(document.getElementById("truck_stick")!.offsetWidth, document.getElementById("truck_stick")!.offsetHeight) / (2/3)),
      restOpacity: 0.8,
      mode: 'static',
      dynamicPage: true,
      position: { left: '50%', top: '65%' },
    } as JoystickManagerOptions);

    truck_stick.on('start', (evt, data) => {
      startPos = data.position;
    });

    truck_stick.on('move', (evt, data) => {
      latestMsg.current = createMsg(frame_id.current, -(data.position.x - startPos.x) / 100, -(data.position.y - startPos.y) / 100);
    });

    truck_stick.on('end', (evt, data) => {
      latestMsg.current = createMsg(frame_id.current, 0, 0);
    });
  });

  let switchMode = (() => {
    if (currentMode === "Remote") {
      setCurrentMode("Auto")
    } else if (currentMode === "Auto") {
      setCurrentMode("Off")
    } else if (currentMode === "Off") {
      setCurrentMode("Remote")
    }
  });

  let publish = (() => {
    let nigger = new Date();
    latestMsg.current.header.stamp.sec = Math.floor(nigger.getTime() / 1000);
    latestMsg.current.header.stamp.nsec = nigger.getMilliseconds() * 1e+6;
    context.publish?.(currentTopic.current, latestMsg.current);
  })

  const panelSettings: SettingsTree = {
    nodes: {
      general: {
        label: "General",
        fields: {
          frame_id: {
            label: "Frame ID",
            input: "string",
            value: settingsFrameId,
          },
          topic: {
            label: "Publication topic",
            input: "string",
            value: settingsTopic,
          },
        },
      },
    },
    actionHandler: (action: SettingsTreeAction) => {
      switch (action.action) {
        case "update":
        if(action.payload.path[0] === "general" && action.payload.path[1] === "frame_id") {
          setFrameId(action.payload.value as string);
        }
        else if(action.payload.path[0] === "general" && action.payload.path[1] === "topic") {
          setTopic(action.payload.value as string);
        }
        break;
      }
    },
  };

  useLayoutEffect(() => {
    context.onRender = (renderState: RenderState, done) => {
      setRenderDone(() => done);
    };

    context.watch("currentFrame");

    context.advertise?.(currentTopic.current, "sensor_msgs/msg/Joy");

    context.setDefaultPanelTitle("truck control")

    let cum = document.getElementById("truck_stick")!.getBoundingClientRect();
    //createJoystick(Math.floor(Math.min(cum.height, cum.width / (2/3))));
    createJoystick();

    setInterval(() => publish(), publishRate);
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  useEffect(() => {
    latestMsg.current.buttons = [0,0,0];
    switch (currentMode) {
      case "Remote":
        latestMsg.current.buttons[controlMap.buttons.remote] = 1;
        break;
      case "Auto":
        latestMsg.current.buttons[controlMap.buttons.auto] = 1;
        break;
      case "Off":
        latestMsg.current.buttons[controlMap.buttons.off] = 1;
        break;
    }
    publish();
    latestMsg.current.buttons = [0,0,0];
  }, [currentMode]);

  useEffect(() => {
    context.updatePanelSettingsEditor(panelSettings);
  }, [panelSettings]);

  useEffect(() => {
    context.unadvertise?.(currentTopic.current);
    currentTopic.current = settingsTopic;
    context.advertise?.(currentTopic.current, "sensor_msgs/msg/Joy");
  }, [settingsTopic])

  useEffect(() => {
    frame_id.current = settingsFrameId;
  }, [settingsFrameId])

  return (
      <div style={{ padding: "1rem" }}>
        <h2>truck control panel</h2>
        <div id="truck_stick"></div>
        <Button variant="outlined" onClick={() => {switchMode();}}>{currentMode}</Button>
      </div>
  );
}

export function initTruckControlPanel(context: PanelExtensionContext): void {
  ReactDOM.render(<TruckControlPanel context={context}/>, context.panelElement);
}
