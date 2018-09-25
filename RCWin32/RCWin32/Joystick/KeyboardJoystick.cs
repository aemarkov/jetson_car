using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace RCWin32.Joystick
{
    /// <summary>
    /// Emulate joystick with keyboard (WSAD)
    /// </summary>
    public class KeyboardJoystick
    {
        private Dictionary<Keys, bool> _pressedKeys;

        public KeyboardJoystick()
        {
            _pressedKeys = new Dictionary<Keys, bool>() { { Keys.W, false }, { Keys.S, false }, { Keys.A, false }, { Keys.D, false } };
        }

        public JoystickInfo SetKeyState(Keys key, bool newState)
        {
            if (!_pressedKeys.ContainsKey(key))
                return Read();

            _pressedKeys[key] = newState;
            return Read();
        }

        public JoystickInfo Read()
        {
            var info = new JoystickInfo();

            if (_pressedKeys[Keys.W])
                info.Y = 0.5f;
            else if (_pressedKeys[Keys.S])
                info.Y = -0.5f;
            else
                info.Y = 0;

            if (_pressedKeys[Keys.A])
                info.Rot = -0.5f;
            else if (_pressedKeys[Keys.D])
                info.Rot = 0.5f;
            else
                info.Rot = 0;

            return info;
        }
    }
}
