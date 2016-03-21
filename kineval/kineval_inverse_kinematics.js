
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics
        in HTML5/JavaScript and threejs

    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/
kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) {
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    // STENCIL: see instructor for random time trial code
}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    //world coordinates of end endeffector
    var worldEndF = matrix_multiply(robot.joints[endeffector_joint].xform, endeffector_position_local);
    //create list of joints from end endeffector to base
    var joints = [];
    joints.push(endeffector_joint);
    var curJoint = endeffector_joint;
    while (robot.joints[curJoint].parent != "base"){
        for (x in robot.joints){
            if (robot.joints[x].child == robot.joints[curJoint].parent){
                joints.push(x)
                curJoint = x;
            }//end if
        }//end innerloop
    }//found base

    //create blank jacobian of 6xn
    var Jacobian = new Array();
    for(var i = 0; i < 6; i++)
        Jacobian[i] = new Array();
    for(var i = 0; i < joints.length; ++i)
        Jacobian[i] = new Array();

        /*for (x in joints){
            console.log("hi");
            console.log(robot.joints[joints[x]].xform);
        }*/
    //loop through joints
    for(var x = 0; x < joints.length; ++x){

        var xform = matrix_copy(robot.joints[joints[x]].xform);

        //compute right side of cross product
        //want: (joint axis) x (end effector position - axis position)
        var crossRight = [];


        //transform axis into world frame
        //axis - origin x end effector
        //zi = joint axis - joint center in world cooridinates

        var right = matrix_multiply(xform, matrix_from_vector([0,0,0,1]));

        var zi = [];
        var axis = matrix_from_vector(robot.joints[joints[x]].axis);

        axis.push([1]);
        var axisPos = matrix_multiply(xform, axis);

        for(var i = 0; i < 3; ++i)
            zi.push(axisPos[i][0] - right[i][0]);

        for(var i = 0; i < 3; ++i)
            crossRight[i] = worldEndF[i][0] - right[i][0];
        //console.log(zi);

        //cross axis of current joint with the right side and
        //push to the current column, then push three zeros for
        //angular displacement. (This assignment doesn't worry about it).
        var column = vector_cross(zi, crossRight);


        for(var i = 0; i < 3; ++i)
            column.push(0);

        //push that column to the Jacobian
        for(var j = 0; j < 6; ++j)
            Jacobian[j].push(column[j]);

    }//end loop through joints

    //create deltaX, differnce between target and position
    //three zeroes at the end for angular displacement
    var dX = [];
    for(var i = 0; i < 6; ++i){
        if(i < 3)
            dX.push(kineval.params.ik_target.position[i][0] - worldEndF[i][0]);
        else
            dX.push(0);
    }

    //turn dX into a matrix for matrix multiply
    var jacob = kineval.params.ik_pseudoinverse?matrix_pseudoinverse(Jacobian):matrix_transpose(Jacobian,4);
    //console.log(Jacobian);
    dX = matrix_from_vector(dX);
    var dTheta = matrix_multiply(jacob, dX);


    //apply control to each joint.
    for(var i = 0; i < joints.length; ++i)
        robot.joints[joints[i]].control = (dTheta[i][0]*kineval.params.ik_steplength);

}//end iterateIK
