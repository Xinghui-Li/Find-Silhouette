#include <tools.hpp>

using namespace cv;
using namespace std;
using namespace Eigen;

// Convert glm vec3 to eigen vector3f
Eigen::Vector3f ConvertGlmToEigen3f( glm::vec3 v ) {
    
    Eigen::Vector3f out_v;
    
    out_v[0] = v.x;
    out_v[1] = v.y;
    out_v[2] = v.z;

    return out_v;

}

// Convert glm mat4 to eigen matrix4f
Eigen::Matrix4f ConvertGlmToEigenMat4f ( glm::mat4 mat ) {
    
    Eigen::Matrix4f out_mat;

    for ( int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
           out_mat(i,j) = mat[j][i];  // In glm::mat4, the first index indicating column and the second indicating row, mat[column][row]. 
    	}                             // In Eigen::Matrix, mat(row, column). 
    }

    return out_mat;

}

// Convert Eigen Matrix4f to glm mat4
glm::mat4 ConvertEigenMat4fToGlm (Eigen::Matrix4f mat){
    
    glm::mat4 out_mat;

    for ( int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
           out_mat[i][j] = mat(j,i);  
        }                             
    }

    return out_mat;

}

// Dehomogeneous the vector from 4 element down to 3.
Eigen::Vector3f pi4to3f ( Eigen::Vector4f v ) {
    
    Eigen::Vector3f out_v;

    out_v[0] = v[0]/v[3];
    out_v[1] = v[1]/v[3];
    out_v[2] = v[2]/v[3];

    return out_v; 

}

// Dehomogeneous the vector from 3 element down to 2.
Eigen::Vector2f pi3to2f ( Eigen::Vector3f v ) {
    
    Eigen::Vector2f out_v;

    out_v[0] = v[0]/v[2];
    out_v[1] = v[1]/v[2];

    return out_v; 

}

// convert a 3D vector to homogeneous vector. 
Eigen::Vector4f pi3to4f ( Eigen::Vector3f v ) {
    
    Eigen::Vector4f out_v;

    out_v[0] = v[0];
    out_v[1] = v[1];
    out_v[2] = v[2];
    out_v[3] = 1;

    return out_v; 

}

// Convert float tp integer based on rounding up if decimals > 0.5 and rounding down if decimals < 0.5
int FloatRoundToInt(float a){
    
    int a_integer = static_cast<int>(a);

    if (a>0) {   	
        if ( a - a_integer < 0.5){

    	    return static_cast<int> (a);

        }else{

    	    return static_cast<int> (a+1);

    	}
    }else{ 
    	if ( a - a_integer < -0.5){

    	    return static_cast<int> (a-1);

        }else{

    	    return static_cast<int> (a);

        }
    }

}

// Search the 3*3 neighbour of the selected pixel (x,y) to see if any of its neighbours is background, return true if it have background in its neighbour. 
// Quite stupid, could find a better way. 
bool SearchNeighbour (cv::Mat image, int x, int y ) {
 
    int count = 0;

	for (int i = -1; i < 2; i++) {
		for (int j = -1; j < 2; j++) {            
            // in opencv index of mat is as (row, column), in another words (y,x)
			if ( image.at<cv::Vec3b>(y+i,x+j)[0] < 50 && 
                 image.at<cv::Vec3b>(y+i,x+j)[1] < 50 &&
                 image.at<cv::Vec3b>(y+i,x+j)[2] < 50 &&
                0 <= x+j && x+j < image.cols && 0 <= y+i && y+i < image.rows){ 

				count++;

			}
		}
	}

	if ( count > 0 ){	

		return true;

	}else{

		return false;

	} 

}

// Draw a point on the image at coordinate (x,y)
void DrawPoint (cv::Mat image, int x , int y ){

	cv::Vec3b Red (0, 0, 255);

    for (int i = -1; i < 2; i++) {
		for (int j = -1; j < 2; j++) {            
            if (0 <= x+j && x+j < image.cols && 0 <= y+i && y+i < image.rows){

			image.at<cv::Vec3b>( y+i, x+j ) = Red;

            }
		}
	}

}

// negate the second and third rows of the matrix. This can be used to convert between opengl and opencv model-pose matrix
Eigen::Matrix4f CVGLConversion ( Eigen::Matrix4f mat ){

    Eigen::Matrix4f out_mat;
    
    out_mat = mat;
    for (int r = 1; r < 3; r++){
        for (int c = 0; c < 4; c++){
            out_mat(r,c) = -mat(r,c);
        }
    }

    return out_mat;

}

// Project the 3D point on the opencv image with defined coordinate in opengl coordinate
Eigen::Vector2i ProjectOnCVimage (int width, int height, Eigen::Matrix4f perspective, Eigen::Matrix4f camera_pose, Eigen::Matrix4f model_pose, Eigen::Vector3f vertex){

    Eigen::Vector4f vertex_hat = pi3to4f(vertex);

    Eigen::Vector4f projection = perspective * camera_pose * model_pose * vertex_hat;
    Eigen::Vector3f x_hat = pi4to3f(projection);
        
    Eigen::Vector2i pixel;    
    pixel[0] = FloatRoundToInt(x_hat[0]*width*0.5+width*0.5);
    pixel[1] = FloatRoundToInt(-x_hat[1]*height*0.5+height*0.5);

    return pixel;
}

// Select 3D point on the silhouette of the image
vector<Vector3f> SelectSilhouettePoint (Mat image, Matrix4f perspective, Matrix4f camera_pose, Matrix4f model_pose, vector<glm::vec3> vertex){

    vector<Vector3f> v_silhouette3d;
    int width = image.cols;
    int height = image.rows;
    
    for (int i = 0; i < vertex.size(); ++i){

        Vector3f v = ConvertGlmToEigen3f(vertex[i]);

        Eigen::Vector2i pixel = ProjectOnCVimage(width, height, perspective, camera_pose, model_pose, v);

        if (SearchNeighbour(image, pixel[0], pixel[1]) == 1){

            v_silhouette3d.push_back(v);
        }
    }
    
    return v_silhouette3d;

}

// Compute distance transform of the input, first denoising and then compute distance transform
Mat DistanceMap ( Mat original, Mat noise ){

    Mat im_temp = original - noise; 
    Mat denoise;

    int morph_elem = 0;
    // morph_size defines the size of the kernel
    int morph_size =2;
    // morph_operator defines the operation
        // 0: MORPH_ERODE
        // 1: MORPH_DILATE
        // 2: MORPH_OPEN
        // 3: MORPH_CLOSE
        // 4: MORPH_GRADIENT
        // 5: MORPH_TOPHAT
        // 6: MORPH_BLACKHAT
        // 7: MORPH_HITMISS
    int morph_operator = 3;
    Mat element = cv::getStructuringElement(morph_elem, cv::Size(2 * morph_size + 1, 2 * morph_size + 1), cv::Point(morph_size, morph_size));
    morphologyEx(im_temp, denoise, morph_operator, element);

    Mat edge;
    Canny(denoise, edge, 0, 255, 3, true);
    Mat binary;
    threshold(edge, binary, 50, 255, cv::THRESH_BINARY_INV);
    Mat dist;
    distanceTransform(binary, dist, DIST_L2, DIST_MASK_PRECISE );
    pow(dist, 0.5, dist);
    dist *=10;
    // Mat dist2;
    // distanceTransform(binary, dist2, DIST_WELSCH, DIST_MASK_PRECISE );
    // Mat dist8u1;
    // normalize(dist, dist8u1, 0.0, 255, NORM_MINMAX, CV_8UC3);
    // dist *= 80;
    // pow(dist, 0.5, dist);
    // Mat dist32s, dist8u1;
    // dist.convertTo(dist32s, CV_32S, 1, 0.5);
    // dist32s &= Scalar::all(255);
    // dist32s.convertTo(dist8u1, CV_8U, 1, 0);
  //   Mat distmap = dist2 - dist;
 	// for(int c=0; c < distmap.cols; c++){
  //       for(int r=243; r < 244; r++)
  //           cout << distmap.at<float>(r,c) << " ";
  //       // cout << endl;
  //   }

    return dist;

}

// Compute the derivative of the distance map    
Eigen::MatrixXf dev_dist( Mat image, int image_x, int image_y){

    Eigen::MatrixXf gradient (1,2);

    gradient(0,0) = (float) (image.at<float>(image_y, image_x + 1) - image.at<float>(image_y, image_x - 1))/2.f;
    gradient(0,1) = (float) (image.at<float>(image_y + 1, image_x) - image.at<float>(image_y - 1, image_x))/2.f;

    return gradient;

}

// Compute the jacobian matrix of the pi3to2 function
Eigen::MatrixXf dev_pi3to2(float x, float y, float z){

    Eigen::MatrixXf out_mat(2,3);

    out_mat(0,0) = 1/z;
    out_mat(0,1) = 0;
    out_mat(0,2) = -x/(z*z);
    out_mat(1,0) = 0;
    out_mat(1,1) = 1/z;
    out_mat(1,2) = -y/(z*z);

    return out_mat;
}

// optimizer calss
// Input: two rotation matrix in opengl frame, one camera intrinsic matrix and 3D silhouette points
// Output: Jacobian matrix, E0 matrix and delta vector

    
optimizer::optimizer(vector<Eigen::Vector3f> v_silhouette3d, Eigen::Matrix3f K, Eigen::Matrix4f camera_pose, Eigen::Matrix4f Model, Mat dist)
: v_silhouette3d(v_silhouette3d), K(K), camera_pose(camera_pose), Model(Model), dist(dist) {
    this->T = CVGLConversion(this->camera_pose * this->Model);
}

Eigen::MatrixXf optimizer::GetE0(){
    
    Eigen::MatrixXf E0(this->v_silhouette3d.size(), 1);

    for (int i = 0; i < this->v_silhouette3d.size(); i++){

        Eigen::Vector3f p = this->v_silhouette3d[i];
        Eigen::Vector3f p_hat = pi4to3f(this->T * pi3to4f(p));
        Eigen::Vector2f x = pi3to2f( this->K * p_hat);

        int image_x = FloatRoundToInt(x[0]);
        int image_y = FloatRoundToInt(x[1]);

        float e0 = (float) this->dist.at<float>(image_y, image_x);
        E0(i,0) = e0;
    }

    return E0;

}

Eigen::MatrixXf optimizer::GetJ(const MatrixXf& E0 ){

    Eigen::MatrixXf J(this->v_silhouette3d.size(), 6);

    for (int i = 0; i < this->v_silhouette3d.size(); i++){

        Eigen::Vector3f p = this->v_silhouette3d[i];
        Eigen::Vector3f p_hat = pi4to3f(this->T * pi3to4f(p));
        Eigen::Vector3f x_hat = this->K * p_hat;
        Eigen::Vector2f x = pi3to2f( x_hat );
        int image_x = FloatRoundToInt(x[0]);
        int image_y = FloatRoundToInt(x[1]);

        Eigen::MatrixXf grad_dist = dev_dist(this->dist, image_x, image_y);
        Eigen::MatrixXf grad_pi = dev_pi3to2(x_hat[0], x_hat[1], x_hat[2]);

        Eigen::MatrixXf gpK = grad_dist * grad_pi * this->K;
        Eigen::Vector3f gpKT = gpK.transpose();
        cout << "-----------------------------------------------------------" << endl;
        cout << "gradient: " << grad_dist << endl;
        cout << "-----------------------------------------------------------" << endl;
        cout << "gpK: " << gpK << endl;
        cout << "-----------------------------------------------------------" << endl;
        cout << "p_hat: " << p_hat.transpose() << endl;
        cout << "-----------------------------------------------------------" << endl;
        Eigen::Vector3f cross = p_hat.cross(gpKT);
        cout << "cross product: " << cross.transpose() << endl;
        cout << "-----------------------------------------------------------" << endl;
        Eigen::MatrixXf jacobian(1,6);
        jacobian << gpK, cross.transpose(); 
        cout << "jacobian: "<< jacobian << endl;
        cout << "-----------------------------------------------------------" << endl;
        cout << "E(i,0): " << E0(i,0) << endl;
        cout << "-----------------------------------------------------------" << endl;

        for (int j = 0; j < 6; j++){
            J(i,j) = jacobian(0,j);
        }

    }

    return J;
}

Eigen::MatrixXf optimizer::GetDelta(){

    Eigen::MatrixXf delta(6,1);
    Eigen::MatrixXf E0 = GetE0();
    Eigen::MatrixXf J = GetJ(E0);
    Eigen::MatrixXf temp = J.transpose()*J; 
    delta = temp.inverse()*J.transpose()*E0;

    return delta;
}
