//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}


    // STENCIL: reference matrix code has the following functions:
    //   matrix_multiply
    function matrix_multiply(A, B){

        var result = [];
        for(var i = 0; i < 4; ++i){
            result[i] = [];
            for(var j = 0; j < 4; ++j){
                var sum = 0;
                for(var k = 0; k < 4; ++k){
                    sum += (A[i][k] * B[k][j]);
                }
                result[i][j] = sum;
            }
        }
        return result;
    }
    //   matrix_transpose
    function matrix_transpose(A){
        var m,n;
        m = A.length;
        n = A[0].length;

        var result = [];

        for(i = 0; i < m; ++i){
            var row = A[i];
            for(j = 0; j < row.length; ++j){
                result[j][i] = row[j];
            }
        }
        return result;

    }

    //   matrix_pseudoinverse (IK)
    //   matrix_invert_affine (IK)

    //   vector_normalize
    function vector_normalize(vec){

        var amplitude = Math.abs(Math.sqrt((vec[0]*vec[0]) + (vec[1]*vec[1]) + (vec[2]*vec[2])));

        vec[0] /= amplitude;
        vec[1] /= amplitude;
        vec[2] /= amplitude;

        return vec;
    }

    //   vector_cross
    function vector_cross(a,b){
        // Check lengths
        if (a.length != 3 || b.length != 3) {
            return;
        }

        return [a[1]*b[2] - a[2]*b[1],
                a[2]*b[0] - a[0]*b[2],
                a[0]*b[1] - a[1]*b[0]];
    }

    //   generate_identity
    function generate_identity(n){
        var mat = [];

        for(i = 0; i < n; ++i){
            mat[i] = [];
            for(j = 0; j < n; ++j){
                if (j == i){
                    mat[j][j] = 1;
                }
                else {
                    mat[i][j] = 0;
                }
            }
        }
        return mat;
    }

    //   generate_translation_matrix
    function generate_translation_matrix(dst){
        
        return [ [1, 0, 0, dst[0] ],
                 [0, 1, 0, dst[1] ],
                 [0, 0, 1, dst[2] ],
                 [0, 0, 0, 1]     ];
    }

    //   generate_rotation_matrix_X
    function generate_rotation_matrix_X(theta){
        return [ [1, 0, 0, 0 ],
                 [0, Math.cos(theta), -1*Math.sin(theta), 0],
                 [0, Math.sin(theta), Math.cos(theta), 0],
                 [0, 0, 0, 1] ];
    }

    //   generate_rotation_matrix_Y
    function generate_rotation_matrix_Y(theta){
        return [ [Math.cos(theta), 0, Math.sin(theta), 0 ],
                 [0, 1, 0, 0],
                 [-1*Math.sin(theta), 0, Math.cos(theta), 0],
                 [0, 0, 0, 1] ];
    }

    //   generate_rotation_matrix_Z
    function generate_rotation_matrix_Z(theta){
        return [ [Math.cos(theta), -1*Math.sin(theta), 0, 0 ],
                 [Math.sin(theta), Math.cos(theta), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1] ];
    }
