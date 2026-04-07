#ifdef USES_P255
//#######################################################################################################
//#################################### Plugin 255: Blinker #############################################
//#######################################################################################################

#define PLUGIN_255
#define PLUGIN_ID_255         255
#define PLUGIN_NAME_255       "Blinker"
#define PLUGIN_VALUENAME1_255 "State"

#include <mcp2515.h>
#define MCP2515_CS 10
#define MCP2515_INT 14

boolean Plugin_255(uint8_t function, struct EventStruct *event, String& string)
{
  boolean success = false;

  switch (function)
  {
    case PLUGIN_DEVICE_ADD:
    {
      auto& dev = Device[++deviceCount];
      dev.Number             = PLUGIN_ID_255;
      dev.Type               = DEVICE_TYPE_SINGLE;
      dev.VType              = Sensor_VType::SENSOR_TYPE_SWITCH;
      dev.ValueCount         = 1;
      dev.SendDataOption     = true;
      dev.TimerOption        = true;
      PCONFIG(1)             = 0x123; // Default CAN ID
      break;
    }

    case PLUGIN_GET_DEVICENAME:
    {
      string = F(PLUGIN_NAME_255);
      break;
    }

    case PLUGIN_GET_DEVICEVALUENAMES:
    {
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[0], PSTR(PLUGIN_VALUENAME1_255));
      break;
    }

    case PLUGIN_WEBFORM_LOAD:
    {
      addFormNumericBox(F("Blink Interval (ms)"), F("p255_interval"), PCONFIG(0), 100, 10000);
      addFormNumericBox(F("CAN ID (hex)"), F("p255_canid"), PCONFIG(1), 0, 0x7FF);
      success = true;
      break;
    }

    case PLUGIN_WEBFORM_SAVE:
    {
      PCONFIG(0) = getFormItemInt(F("p255_interval"));
      PCONFIG(1) = getFormItemInt(F("p255_canid"));
      success = true;
      break;
    }

    case PLUGIN_INIT:
    {
      if (CONFIG_PIN1 != -1) {
        pinMode(CONFIG_PIN1, OUTPUT);
      }
      success = true;
      break;
    }

    case PLUGIN_TEN_PER_SECOND:
    {
      static unsigned long nextBlink = 0;
      static bool ledState = false;
      
      if (CONFIG_PIN1 != -1 && PCONFIG(0) > 0) {
        if (millis() >= nextBlink) {
          nextBlink = millis() + PCONFIG(0);
          ledState = !ledState;
          digitalWrite(CONFIG_PIN1, ledState ? HIGH : LOW);
          UserVar.setFloat(event->TaskIndex, 0, ledState);
        }
      }
      success = true;
      break;
    }
  }
  return success;
}
#endif
