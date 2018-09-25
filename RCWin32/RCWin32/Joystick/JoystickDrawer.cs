using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace RCWin32.Joystick
{
    public class JoystickDrawer : Panel
    {
        private const int PADDING = 8;
        private const int BAR_WIDTH = 32;
        private const int POINTER_SIZE = 16;
        private const int ROT_POINTER_WIDTH = 8;
        private Pen _pen;
        private Brush _brush;

        public JoystickDrawer()
        {
            _pen = Pens.Blue;
            _brush = new SolidBrush(Color.Blue);
            DoubleBuffered = true;
        }


        public float X { get; set; }
        public float Y { get; set; }
        public float Rot { get; set; }
        public float Throttle { get; set; }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);

            e.Graphics.Clear(Color.White);

            //e.Graphics.DrawLine(_pen, 0, PADDING, Width, PADDING);
            //e.Graphics.DrawLine(_pen, 0, Height - PADDING, Width, Height - PADDING);
            //e.Graphics.DrawLine(_pen, PADDING, 0, PADDING, Height);
            //e.Graphics.DrawLine(_pen, Width - PADDING, 0, Width - PADDING, Height);

            DrawThrottle(e.Graphics, Throttle);
            DrawRot(e.Graphics, Rot);
            DrawXY(e.Graphics, X, Y);            
        }

        private void DrawThrottle(Graphics g, float throttle)
        {
            if (throttle < 0) throttle = 0;
            else if (throttle > 1) throttle = 1;

            int fullHeight = Height - 3 * PADDING - BAR_WIDTH;
            float height = throttle * fullHeight;

            g.DrawRectangle(_pen, Width - PADDING - BAR_WIDTH, PADDING, BAR_WIDTH, fullHeight);
            g.FillRectangle(_brush, Width - PADDING - BAR_WIDTH, PADDING + (fullHeight - height), BAR_WIDTH, height);
        }

        private void DrawRot(Graphics g, float rot)
        {
            int fullWidth = Width - 3 * PADDING - BAR_WIDTH;
            int fullWidthFix = fullWidth -  ROT_POINTER_WIDTH;
            float center = PADDING + fullWidth / 2;
            float x = center + fullWidthFix * rot;

            g.DrawRectangle(_pen, PADDING, Height - PADDING - BAR_WIDTH, fullWidth, BAR_WIDTH);            
            g.FillRectangle(_brush, x - ROT_POINTER_WIDTH / 2, Height - PADDING - BAR_WIDTH, ROT_POINTER_WIDTH, BAR_WIDTH);
        }

        private void DrawXY(Graphics g, float x, float y)
        {
            int width = Width - 3 * PADDING - BAR_WIDTH;                // area width
            int height = Height - 3 * PADDING - BAR_WIDTH;              // area height
            int widthFix = width - POINTER_SIZE;
            int heightFix = height - POINTER_SIZE;
            int centerX = PADDING + width / 2;
            int centerY = PADDING + height / 2;

            float markerX = centerX + widthFix * x;
            float markerY = centerX - widthFix * y;

            g.DrawRectangle(_pen, PADDING, PADDING, width, height);
            g.FillEllipse(_brush, markerX - POINTER_SIZE / 2, markerY - POINTER_SIZE / 2, POINTER_SIZE, POINTER_SIZE);
        }

        // clip to [-0.5; 0.5]
        private float Clip(float value)
        {
            if (value < -0.5) return -0.5f;
            if (value > 0.5) return 0.5f;
            return value;
        }
    }    
}
