using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace RCWin32
{
    public partial class Form1 : Form
    {
        enum Mode
        {
            JOYSTICK,
            KEYBOARD
        };

        Joystick _joystick;
        KeyboardJoystick _keyboardJoystick;
        JoystickInfo _cmd;
        Mode _mode;

        public Form1()
        {
            InitializeComponent();
            numJoysticId.Maximum = Joystick.GetDeviceCount();
            
            _joystick = new Joystick(0);
            _keyboardJoystick = new KeyboardJoystick();
            SetMode(Mode.JOYSTICK);

            KeyPreview = true;
            KeyDown += Form1_KeyDown;
            KeyUp += Form1_KeyUp;
        }        

        // Set joystick mode
        private void btnJoystick_Click(object sender, EventArgs e)
        {
            SetMode(Mode.JOYSTICK);
        }

        // Set keyboard mode
        private void button1_Click(object sender, EventArgs e)
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
            _cmd = _joystick.Read();
            DrawJoystick(_cmd);
        }

        private void Form1_KeyDown(object sender, KeyEventArgs e)
        {
            if (_mode == Mode.KEYBOARD)
            {
                _cmd = _keyboardJoystick.SetKeyState(e.KeyCode, true);
                DrawJoystick(_cmd);
            }

        }

        private void Form1_KeyUp(object sender, KeyEventArgs e)
        {
            if (_mode == Mode.KEYBOARD)
            {
                _cmd = _keyboardJoystick.SetKeyState(e.KeyCode, false);
                DrawJoystick(_cmd);
            }
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
                joystickUpdateTime.Enabled = true;
            }
            else
            {
                btnJoystick.BackColor = SystemColors.Control;
                btnKeyboard.BackColor = Color.LightSkyBlue;
                gboxJoystick.Visible = false;
                gboxKeyboard.Visible = true;
                joystickUpdateTime.Enabled = false;
                _cmd = _keyboardJoystick.Read();
                DrawJoystick(_cmd);
            }
        }

    }
}
