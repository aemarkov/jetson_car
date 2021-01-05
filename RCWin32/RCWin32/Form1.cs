using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace RCWin32
{
    public partial class Form1 : Form
    {
        UdpSender _sender;
        sbyte left, right;

        public Form1()
        {
            InitializeComponent();

            KeyPreview = true;
            KeyDown += Form1_KeyDown;
            KeyUp += Form1_KeyUp;

            joystickControl.JoystickMoved += JoystickControl_JoystickMoved;
            _sender = new UdpSender();

            joystickControl.Focus();
        }

        private void Form1_KeyUp(object sender, KeyEventArgs e)
        {
            joystickControl.KeyChanged(e.KeyCode, false);
        }

        private void Form1_KeyDown(object sender, KeyEventArgs e)
        {
            joystickControl.KeyChanged(e.KeyCode, true);
        }
        private void JoystickControl_JoystickMoved(object sender, Joystick.JoystickInfo e)
        {
            float forward = e.Y;
            float rot = e.Rot;

            sbyte left, right;
            int maxPWM = (int)numMaxPWM.Value;

            left = (sbyte)(2 * forward * maxPWM);
            right = (sbyte)(2 * forward * maxPWM);

            left -= (sbyte)(2 * rot * maxPWM);
            right += (sbyte)(2* rot * maxPWM);

            lblLeft.Text = left.ToString();
            lblRight.Text = right.ToString();

            _sender.Send(left, right);

        }

        private void btnStart_Click(object sender, EventArgs e)
        {
            if (!int.TryParse(txtPort.Text, out var port))
                return;

            if (String.IsNullOrWhiteSpace(txtHost.Text))
                return;

            _sender.SetRemote(txtHost.Text, Int32.Parse(txtPort.Text));
        }
    }
}
