using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;

namespace RCWin32
{
    /// <summary>
    /// Read joystick without DirectX
    /// </summary>
    public class Joystick
    {
        public Joystick(Int32 uJoyID)
        {
            _info = new JOYINFOEX();
            _info.dwSize = 13 * sizeof(Int32);
            _info.dwFlags = JOY_RETURNALL;
            _uJoyID = uJoyID;
        }

        public static UInt32 GetDeviceCount()
        {
            return joyGetNumDevs();
        }

        public void SetJoyId(Int32 uJoyId)
        {
            _uJoyID = uJoyId;
        }

        public JoystickInfo Read()
        {
            joyGetPosEx(_uJoyID, ref _info);
            JoystickInfo info = new JoystickInfo()
            {
                X = (_info.dwXpos - 32768) / 65536.0f,
                Y = (-_info.dwYpos + 32768) / 65536.0f,
                Rot = (_info.dwRpos - 32768) / 65536.0f,
                Throttle = (-_info.dwZpos + 65536) / 65536.0f,
                POV = (_info.dwPOV == 65535) ? -1 : (_info.dwPOV / 100.0f),
                Buttons = _info.dwButtons
            };

            return info;
        }

        private Int32 _uJoyID;
        private JOYINFOEX _info;

        private static Int32 JOY_RETURNX = 0x00000001;
        private static Int32 JOY_RETURNY = 0x00000002;
        private static Int32 JOY_RETURNZ = 0x00000004;
        private static Int32 JOY_RETURNR = 0x00000008;
        private static Int32 JOY_RETURNU = 0x00000010;
        private static Int32 JOY_RETURNV = 0x00000020;
        private static Int32 JOY_RETURNPOV = 0x00000040;
        private static Int32 JOY_RETURNBUTTONS = 0x00000080;
        private static Int32 JOY_RETURNALL = (JOY_RETURNX | JOY_RETURNY | JOY_RETURNZ | JOY_RETURNR | JOY_RETURNU | JOY_RETURNV | JOY_RETURNPOV | JOY_RETURNBUTTONS);

        private struct JOYINFOEX
        {
            public Int32 dwSize;
            public Int32 dwFlags;
            public Int32 dwXpos;
            public Int32 dwYpos;
            public Int32 dwZpos;
            public Int32 dwRpos;
            public Int32 dwUpos;
            public Int32 dwVpos;
            public Int32 dwButtons;
            public Int32 dwButtonNumber;
            public Int32 dwPOV;
            public Int32 dwReserved1;
            public Int32 dwReserved2;
        }

        [DllImport("winmm.dll", CallingConvention = CallingConvention.StdCall)]
        private static extern Int32 joyGetPosEx(Int32 uJoyID, ref JOYINFOEX pji);

        [DllImport("winmm.dll", CallingConvention = CallingConvention.StdCall)]
        private static extern UInt32 joyGetNumDevs();
    }

    public struct JoystickInfo
    {
        /// <summary>
        /// Stick left-right
        /// </summary>
        public float X;

        /// <summary>
        /// Stick forward-backward
        /// </summary>
        public float Y;

        /// <summary>
        /// Rotate stick
        /// (also known as rudder)
        /// </summary>
        public float Rot;

        /// <summary>
        /// Throrttle control
        /// (also known as z-axis)
        /// </summary>
        public float Throttle;

        /// <summary>
        /// Position of point-of-view control in degrees,
        /// -1 if POV control in center position
        /// </summary>
        public float POV;

        /// <summary>
        /// Bit mask of pressed buttons
        /// </summary>
        public Int32 Buttons;

        /// <summary>
        /// Check button with given number
        /// </summary>
        /// <param name="number">Button number [1; 32]</param>
        /// <returns></returns>
        public bool GetButton(byte number)
        {
            if (number < 1 || number > 32)
                throw new ArgumentOutOfRangeException();

            return (Buttons & (1 << (number - 1))) != 0;
        }
    }
}
