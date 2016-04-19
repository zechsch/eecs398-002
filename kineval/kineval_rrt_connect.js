
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | RRT motion planning

    Implementation of robot kinematics, control, decision making, and dynamics
        in HTML5/JavaScript and threejs

    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// STUDENT:
// compute motion plan and output into robot_path array
// elements of robot_path are vertices based on tree structure in tree_init()
// motion planner assumes collision checking by kineval.poseIsCollision()

/* KE 2 : Notes:
   - Distance computation needs to consider modulo for joint angles
   - robot_path[] should be used as desireds for controls
   - Add visualization of configuration for current sample
*/

/*
STUDENT: reference code has functions for:

*/

kineval.planMotionRRTConnect = function motionPlanningRRTConnect() {

    // exit function if RRT is not implemented
    //   start by uncommenting kineval.robotRRTPlannerInit
    if (typeof kineval.robotRRTPlannerInit === 'undefined') return;

    if ((kineval.params.update_motion_plan) && (!kineval.params.generating_motion_plan)) {
        kineval.robotRRTPlannerInit();
        kineval.params.generating_motion_plan = true;
        kineval.params.update_motion_plan = false;
        kineval.params.planner_state = "initializing";
    }
    if (kineval.params.generating_motion_plan) {
        rrt_result = robot_rrt_planner_iterate();
        if (rrt_result === "reached") {
            kineval.params.update_motion_plan = false; // KE T needed due to slight timing issue
            kineval.params.generating_motion_plan = false;
            textbar.innerHTML = "planner execution complete";
            kineval.params.planner_state = "complete";
        }
        else kineval.params.planner_state = "searching";
    }
    else if (kineval.params.update_motion_plan_traversal||kineval.params.persist_motion_plan_traversal) {

        if (kineval.params.persist_motion_plan_traversal) {
            kineval.motion_plan_traversal_index = (kineval.motion_plan_traversal_index+1)%kineval.motion_plan.length;
            textbar.innerHTML = "traversing planned motion trajectory";
        }
        else
            kineval.params.update_motion_plan_traversal = false;

        // set robot pose from entry in planned robot path
        robot.origin.xyz = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[0],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[1],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[2]
        ];

        robot.origin.rpy = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[3],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[4],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[5]
        ];

        // KE 2 : need to move q_names into a global parameter
        for (x in robot.joints) {
            robot.joints[x].angle = kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]];
        }

    }
}


    // STENCIL: uncomment and complete initialization function
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];


    epsLin = .5;
    epsAng = .5;

    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints)
        q_start_config.push(robot.joints[x].angle);


    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
    }

    // set goal configuration as the zero configuration
    var i;
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    start = tree_init(q_start_config);
    end = tree_init(q_goal_config);
    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    path = [];
    finished = false;

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();
}



function robot_rrt_planner_iterate() {

    var i;
    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

    // STENCIL: implement single rrt iteration here. an asynch timing mechanism
    //   is used instead of a for loop to avoid blocking and non-responsiveness
    //   in the browser.
    //
    //   once plan is found, highlight vertices of found path by:
    //     tree.vertices[i].vertex[j].geom.material.color = {r:1,g:0,b:0};
    //
    //   provided support functions:
    //
    //   kineval.poseIsCollision - returns if a configuration is in collision
    //   tree_init - creates a tree of configurations
    //   tree_add_vertex - adds and displays new configuration vertex for a tree
    //   tree_add_edge - adds and displays new tree edge between configurations

        q_rand = random_config();
        if(rrt_extend(start, q_rand) != "trapped"){
            if(calcDist(start, end)){
                finalVert = start.newest;
                tree_add_vertex(start, b);
                tree_add_edge(start, finalVert, start.newest);
                rrt_iterate = false;
                find_path(start.vertices[0], start.vertices[start.newest]);
                finished = false;
                midpoint.vertex[midpoint.vertex.length-2] = 0;
                find_path(midpoint, end.vertices[0]);
                console.log(1);
                for(x in kineval.motion_plan)
                    kineval.motion_plan[x].geom.material.color = {r:0,g:0,b:255};
                return "reached";
            }
        }

        q_rand = random_config();
        if(rrt_extend(end, q_rand) != "trapped" && rrt_iterate){
            if(calcDist(end,start)){
                finalVert = end.newest;
                tree_add_vertex(end, b);
                tree_add_edge(end, finalVert, end.newest);
                rrt_iterate = false;
                find_path(start.vertices[0], midpoint);
                finished = false;
                midpoint.vertex[midpoint.vertex.length-2] = 0;
                find_path(end.vertices[end.newest], end.vertices[0]);
                console.log(2);
                for(x in kineval.motion_plan)
                    kineval.motion_plan[x].geom.material.color = {r:255,g:0,b:0};
                return "reached";
            }
        }
    }
}

//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}

function tree_add_vertex(tree,q) {


    // create new vertex object for tree with given configuration and no edges
    var new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;
    new_vertex.vertex.push(0);

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
}

function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);
    vertex.geom = temp_mesh;
}


function tree_add_edge(tree,q1_idx,q2_idx) {

    // add edge to first vertex as pointer to second vertex
    tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);

    // add edge to second vertex as pointer to first vertex
    tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);

    // can draw edge here, but not doing so to save rendering computation
}


//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

function random_config(){
    var xval = Math.abs(robot_boundary[0][0] - robot_boundary[1][0]);
    var zval = Math.abs(robot_boundary[0][2] - robot_boundary[1][2]);
    var q = [];
    q[0] = Math.random() * (xval + 1) + robot_boundary[0][0];
    q[1] = 0;
    q[2] = Math.random() * (zval + 1) + robot_boundary[0][2];
    q[3] = 0;
    q[4] = Math.random() * 2 * Math.PI;
    q[5] = 0;
    q[6] = Math.random() * 2 * Math.PI;
    q[7] = Math.random() * 2 * Math.PI;
    q[8] = Math.random() * 2 * Math.PI;
    q[9] = Math.random() * 2 * Math.PI;
    q[10] = Math.random() * 2 * Math.PI;

    return q;
}

function rrt_extend(T, q){
    var q_near_idx = nearest_neighbor(T,q);
    q_new = new_config(T,q,q_near_idx);


    if(!kineval.poseIsCollision(q_new)){
        rrt_iter_count++;
        tree_add_vertex(T,q_new);
        tree_add_edge(T,q_near_idx, T.newest);

        return "advanced";
    }
    return "trapped";
}

function nearest_neighbor(T,q){
    var closestDist = Number.MAX_SAFE_INTEGER;
    var closest = 0;
    //loop through vertices
    for(var i = 0; i < T.vertices.length; ++i){
        var xDist = q[0] - T.vertices[i].vertex[0];
        var zDist = q[2] - T.vertices[i].vertex[2];
        var linDist = Math.sqrt(Math.pow(xDist, 2) + Math.pow(zDist,2));
        var ang = 0;
        for(var j = 4; j < T.vertices[i].length; ++j)
            ang += (q[j] - T.vertices[i].vertex[j])

        if((linDist + ang) < closestDist){
            closest = i;
            closestDist = linDist + ang;
        }
    }
    return closest;
}

function new_config(T, q, q_near){
    var q_n = [];
    var x = q[0] - T.vertices[q_near].vertex[0];
    var z = q[2] - T.vertices[q_near].vertex[2];
    var linMagnitude = Math.sqrt(Math.pow(x,2) + Math.pow(z,2));
    x /= linMagnitude;
    x *= epsLin;
    z /= linMagnitude;
    z *= epsLin;
    x += T.vertices[q_near].vertex[0];
    z += T.vertices[q_near].vertex[2];
    for(var i = 0; i < 11; ++i){
        if(i == 0) q_n[i] = x;
        else if (i == 2) q_n[i] = z;
        else if (i == 1 || i == 3 || i == 5) q_n[i] = 0;
        else{
            var diff = q[i] - T.vertices[q_near].vertex[i];
            q_n[i] = epsAng * diff;
        }
    }

    return q_n;
}

function calcDist(A,B){
    var a = A.vertices[A.newest].vertex;
    var closest = nearest_neighbor(B, A.vertices[A.newest].vertex);
    b = B.vertices[closest].vertex;

    if(Math.sqrt(
        Math.pow(b[0] - a[0], 2) +
        Math.pow(b[2] - a[2], 2)) > epsLin) return false;
    for(var i = 4; i < 11; ++i){
        if(i == 5) continue;
        if(b[i] - a[i] > epsAng) return false;
    }
    midpoint = B.vertices[closest];
    return true;

}

function find_path(a,b){

    if(a.vertex[11] == 1) return;
    a.vertex[11] = 1;
    kineval.motion_plan.push(a);

    if(a==b || finished){
        finished = true;
        return kineval.motion_plan;
    }

    for(neighbor in a.edges){
        if(a.edges[neighbor].vertex[11] != 1){
            kineval.motion_plan.push(a.edges[neighbor]);
            find_path(a.edges[neighbor], b);
            if(finished) return kineval.motion_plan;
            kineval.motion_plan.pop();
        }
    }
    if(!finished) kineval.motion_plan.pop();

}
