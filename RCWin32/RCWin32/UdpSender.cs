using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;

namespace RCWin32
{
    /// <summary>
    /// Sends commands to robot
    /// </summary>
    public class UdpSender
    {
        Socket _socket;
        byte[] _buffer;
        EndPoint _remoteEP;

        public UdpSender()
        {
            _socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            _buffer = new byte[2];
        }

        public void SetRemote(string host, int port)
        {
            _remoteEP = new IPEndPoint(IPAddress.Parse(host), port);
        }

        public void Send(sbyte left, sbyte right)
        {
            if (_remoteEP == null) return;

            _buffer[0] = (byte)left;
            _buffer[1] = (byte)right;
            _socket.SendTo(_buffer, _remoteEP);

        }
    }
}
