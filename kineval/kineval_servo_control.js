
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics
        in HTML5/JavaScript and threejs

    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

/*desired setpoints:
[{"clavicle_right_yaw":0,"shoulder_right_yaw":0,"upperarm_right_pitch":0,"forearm_right_yaw":0,"clavicle_left_roll":0},{"clavicle_right_yaw":0,"shoulder_right_yaw":1.5900000000000012,"upperarm_right_pitch":1.7200000000000013,"forearm_right_yaw":-1.2100000000000009,"clavicle_left_roll":0},{"clavicle_right_yaw":0,"shoulder_right_yaw":-0.9000000000000007,"upperarm_right_pitch":1.7200000000000013,"forearm_right_yaw":-4.339999999999952,"clavicle_left_roll":0},{"clavicle_right_yaw":0,"shoulder_right_yaw":-0.9000000000000007,"upperarm_right_pitch":0.1299999999999999,"forearm_right_yaw":-4.41999999999995,"clavicle_left_roll":0},{"clavicle_right_yaw":0,"shoulder_right_yaw":-0.9000000000000007,"upperarm_right_pitch":-0.9400000000000006,"forearm_right_yaw":-7.23999999999989,"clavicle_left_roll":0},{"clavicle_right_yaw":0,"shoulder_right_yaw":-2.479999999999991,"upperarm_right_pitch":-0.9400000000000006,"forearm_right_yaw":-7.23999999999989,"clavicle_left_roll":0},{"clavicle_right_yaw":0,"shoulder_right_yaw":-3.319999999999973,"upperarm_right_pitch":-0.04999999999999992,"forearm_right_yaw":-6.429999999999907,"clavicle_left_roll":0},{"clavicle_right_yaw":-4.609999999994686,"shoulder_right_yaw":-9.499999999993886,"upperarm_right_pitch":-1.560000000000001,"forearm_right_yaw":-7.199999999998305,"clavicle_left_roll":0},{"clavicle_right_yaw":-4.609999999999946,"shoulder_right_yaw":-8.539999999999862,"upperarm_right_pitch":-0.04999999999999992,"forearm_right_yaw":-7.819999999999878,"clavicle_left_roll":0},{"clavicle_right_yaw":-4.609999999999946,"shoulder_right_yaw":-8.539999999999862,"upperarm_right_pitch":-1.2500000000000009,"forearm_right_yaw":-6.4699999999999065,"clavicle_left_roll":0}]
*/

kineval.setpointDanceSequence = function execute_setpoints() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) return;
    //kineval.params.persist_pd = true;

    var sum = 0;
    // STENCIL: implement FSM to cycle through dance pose setpoints
    for(x in robot.joints){
        kineval.params.setpoint_target[x] = kineval.setpoints[kineval.params.dance_pose_index][x];
        sum += Math.pow((kineval.params.setpoint_target[x]-robot.joints[x].angle), 2);
    }
    var root = Math.sqrt(sum);

    if(root >= 0.01)
        return;

    kineval.params.dance_pose_index += 1;

    if(kineval.params.dance_pose_index == 10)
        kineval.params.dance_pose_index = 0;


    kineval.setPoseSetpoint(kineval.params.dance_pose_index);
}

kineval.setpointClockMovement = function execute_clock() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_clock) return;

    var curdate = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = curdate.getSeconds()/60*2*Math.PI;
    }
}


kineval.robotArmControllerSetpoint = function robot_pd_control () {

    // if update not requested, exit routine
    if ((!kineval.params.update_pd)&&(!kineval.params.persist_pd)) return;

    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control

    // STENCIL: implement P servo controller over joints
    for (x in robot.joints){
        var error = kineval.params.setpoint_target[x] - robot.joints[x].angle;

        robot.joints[x].control = (robot.joints[x].servo.p_gain * error);
    }
}
