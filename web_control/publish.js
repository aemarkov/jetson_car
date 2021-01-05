
// TODO: don't use hard-coded hostname
const ROSBRIDGE_URL = 'ws://jetson:9090'

const joy = new JoyStick('joyDiv');
let publish_handler = undefined;
status_lbl = document.getElementById('status')

connect_ros(ROSBRIDGE_URL);

function send(topic) {
    var msg = new ROSLIB.Message({
        header: {},
        axes: [
            0,                            // dummy
            parseFloat(joy.GetX())/100.0, // forward (speed)
            parseFloat(joy.GetY())/100.0  // rotation
        ],
        buttons: []
    });
    topic.publish(msg);
}

function display_status(status) {
    status_lbl.innerHTML = status
}

function connect_ros(url) {
    const ros = new ROSLIB.Ros({
        url: url
    });

    ros.on('connection', function () {
        console.log('Connected to websocket server.');
        const topic = new ROSLIB.Topic({
            ros: ros,
            name: '/joy',
            messageType: 'sensor_msgs/Joy'
        });
        publish_handler = setInterval(() => send(topic), 50);
        console.log('Start publishing')
        display_status('Connected')
    });

    ros.on('error', function (error) {
        console.log('Error connecting to websocket server: ', error);
        display_status('No connection')
    });

    ros.on('close', function () {
        console.log('Connection to websocket server closed.');
        if (publish_handler !== undefined) {
            clearInterval(publish_handler)
            console.log('Stop publishing')
            display_status('No connection')
        }
    });

}
