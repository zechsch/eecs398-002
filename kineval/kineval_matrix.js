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

        var ma,na,mb,nb;
        ma = A.length;
        na = A[0].length;
        mb = B.length;
        nb = B[0].length;

        if(na != mb){
            throw "Error: incompatible sizes."
        }

        var result = [];
        for(i = 0; i < ma; ++i){
            result[i] = [];
            for(j = 0; j < nb; ++j){
                var sum = 0;
                for(k = 0; k < na; ++k){
                    sum += A[i][k] * B[k][j];
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

        var current = 0;
        var mat = [];

        for(i = 0; i < n; ++i){
            mat[i] = [];
            for(j = 0; j < n; ++j){
                if (j == current){
                    mat[i][j] = 1;
                    ++current;
                }
                else {
                    mat[i][j] = 0;
                }
            }
        }
    }

    //   generate_translation_matrix
    //   generate_rotation_matrix_X
    //   generate_rotation_matrix_Y
    //   generate_rotation_matrix_Z
