//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES
//////////////////////////////////////////////////

    // STENCIL: reference quaternion code has the following functions:
    //   quaternion_from_axisangle
    function quaternion_from_axisangle(axis, angle){
        //[cos(Θ/2), ux sin(Θ/2), uy sin(Θ/2), uz sin(Θ/2)]
        var x = axis[0];
        var y = axis[1];
        var z = axis[2];
        return [Math.cos(angle/2), x*Math.sin(angle/2), y*Math.sin(angle/2), z*Math.sin(angle/2)];
    }

    //   quaternion_normalize (done)
    function quaternion_normalize(quat){
        var scale = Math.sqrt(Math.pow(quat[0],2) + Math.pow(quat[1],2) + Math.pow(quat[2],2) + Math.pow(quat[3],2));
        for(var i = 0; i < quat.length; ++i)
            quat[i] /= scale;

        return quat;
    }

    //   quaternion_to_rotation_matrix (Done)
    function quaternion_to_rotation_matrix(quat){
        var a = quat[0];
        var b = quat[1];
        var c = quat[2];
        var d = quat[3];

        var rows = new Array(4);
        for(var i = 0; i < 4; ++i){
            rows[i] = new Array(4);
        }

        rows[0][0] = Math.pow(a,2) + Math.pow(b,2) - Math.pow(c, 2) - Math.pow(d,2);
        rows[0][1] = 2 * (b*c-a*d);
        rows[0][2] = 2 * (a*c+b*d);
        rows[0][3] = 0;
        rows[1][0] = 2 * (b*c+a*d);
        rows[1][1] = Math.pow(a,2) - Math.pow(b,2) + Math.pow(c,2) - Math.pow(d,2);
        rows[1][2] = 2 * (c*d - a*b);
        rows[1][3] = 0;
        rows[2][0] = 2 * (b*d - a*c);
        rows[2][1] = 2 * (a*b + c*d);
        rows[2][2] = Math.pow(a,2) - Math.pow(b,2) - Math.pow(c,2) + Math.pow(d,2);
        rows[2][3] = 0;
        rows[3][0] = 0;
        rows[3][1] = 0;
        rows[3][2] = 0;
        rows[3][3] = 1;

        return rows;



    }

    //   quaternion_multiply (DONE)
    function quaternion_multiply(q1, q2){

        var a = q1[0];
        var b = q1[1];
        var c = q1[2];
        var d = q1[3];
        var e = q2[0];
        var f = q2[1];
        var g = q2[2];
        var h = q2[3];

        var toReturn = new array(4);
        toReturn[0] = (a*e) - (b*f) - (c*g) - (d*h);
        toReturn[1] = (a*f) + (b*e) + (c*h) - (d*g);
        toReturn[2] = (a*g) - (b*h) + (c*e) + (d*f);
        toReturn[3] = (a*h) + (b*g) - (c*f) + (d*e);
        return toReturn;
    }

    // 1 2 3 to matrix stack
