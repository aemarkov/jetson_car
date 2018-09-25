using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace RCWin32.Joystick
{
    public partial class JoystickControl : UserControl
    {
        public event EventHandler<JoystickInfo> JoystickMoved;

        enum Mode
        {
            JOYSTICK,
            KEYBOARD
        };

        Joystick _joystick;
        KeyboardJoystick _keyboardJoystick;        
        Mode _mode;

        public JoystickControl()
        {
            InitializeComponent();

            numJoysticId.Maximum = Joystick.GetDeviceCount();
            _joystick = new Joystick(0);
            _keyboardJoystick = new KeyboardJoystick();
            SetMode(Mode.JOYSTICK);

        }

        // Key pressed (key press/up can handle only form)
        public void KeyChanged(Keys key, bool pressed)
        {
            if (_mode == Mode.KEYBOARD)
            {
                var cmd = _keyboardJoystick.SetKeyState(key, pressed);
                JoystickMoved?.Invoke(this, cmd);
                DrawJoystick(cmd);
            }
        }

        // Set joystick mode
        private void btnJoystick_Click(object sender, EventArgs e)
        {
            SetMode(Mode.JOYSTICK);
        }

        // Set keyboard mode
        private void btnKeyboard_Click(object sender, EventArgs e)
        {
            SetMode(Mode.KEYBOARD);
        }

        // Select joystick ID
        private void numJoysticId_ValueChanged(object sender, EventArgs e)
        {
            _joystick.SetJoyId((Int32)numJoysticId.Value);
        }


        // Joystick update time
        private void timer1_Tick(object sender, EventArgs e)
        {
            var cmd = _joystick.Read();
            JoystickMoved?.Invoke(this, cmd);
            DrawJoystick(cmd);
        }       

        private void DrawJoystick(JoystickInfo info)
        {
            joystickDrawer.Throttle = info.Throttle;
            joystickDrawer.Rot = info.Rot;
            joystickDrawer.X = info.X;
            joystickDrawer.Y = info.Y;

            joystickDrawer.Invalidate();
        }

        private void SetMode(Mode mode)
        {
            _mode = mode;
            if (mode == Mode.JOYSTICK)
            {
                btnJoystick.BackColor = Color.LightSkyBlue;
                btnKeyboard.BackColor = SystemColors.Control;
                gboxJoystick.Visible = true;
                gboxKeyboard.Visible = false;
                joystickUpdateTimer.Enabled = true;
            }
            else
            {
                btnJoystick.BackColor = SystemColors.Control;
                btnKeyboard.BackColor = Color.LightSkyBlue;
                gboxJoystick.Visible = false;
                gboxKeyboard.Visible = true;
                joystickUpdateTimer.Enabled = false;
                var cmd = _keyboardJoystick.Read();
                JoystickMoved?.Invoke(this, cmd);
                DrawJoystick(cmd);
            }
        }
    }    
}
