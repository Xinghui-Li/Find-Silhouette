#include <tools.hpp>
#include <iomanip>

using namespace cv;
using namespace std;
using namespace Eigen;

// Convert glm vec3 to eigen Vector3d
Eigen::Vector3d ConvertGlmToEigen3f( const glm::vec3& v ) {
    
    Eigen::Vector3d out_v;
    
    out_v[0] = v.x;
    out_v[1] = v.y;
    out_v[2] = v.z;

    return out_v;

}

// Convert glm mat4 to eigen Matrix4d
Eigen::Matrix4d ConvertGlmToEigenMat4f (const glm::mat4& mat ) {
    
    Eigen::Matrix4d out_mat;

    for ( int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
           out_mat(i,j) = mat[j][i];  // In glm::mat4, the first index indicating column and the second indicating row, mat[column][row]. 
    	}                             // In Eigen::Matrix, mat(row, column). 
    }

    return out_mat;

}

// Convert Eigen Matrix4d to glm mat4
glm::mat4 ConvertEigenMat4fToGlm (const Eigen::Matrix4d& mat){
    
    glm::mat4 out_mat;

    for ( int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
           out_mat[i][j] = mat(j,i);  
        }                             
    }

    return out_mat;

}

glm::vec3 ConvertEigen3dToGlm (const Vector3d& v ){

    glm::vec3 out_v;
    
    out_v.x = v[0];
    out_v.y = v[1];
    out_v.z = v[2];

    return out_v;

}

// Dehomogeneous the vector from 4 element down to 3.
Eigen::Vector3d pi4to3f ( const Eigen::Vector4d& v ) {
    
    Eigen::Vector3d out_v;

    out_v[0] = v[0]/v[3];
    out_v[1] = v[1]/v[3];
    out_v[2] = v[2]/v[3];

    return out_v; 

}

// Dehomogeneous the vector from 3 element down to 2.
Eigen::Vector2d pi3to2f ( const Eigen::Vector3d& v ) {
    
    Eigen::Vector2d out_v;

    out_v[0] = v[0]/v[2];
    out_v[1] = v[1]/v[2];

    return out_v; 

}

// convert a 3D vector to homogeneous vector. 
Eigen::Vector4d pi3to4f ( const Eigen::Vector3d& v ) {
    
    Eigen::Vector4d out_v;

    out_v[0] = v[0];
    out_v[1] = v[1];
    out_v[2] = v[2];
    out_v[3] = 1;

    return out_v; 

}

Matrix4d pseudo_exp( const VectorXd& delta ){

    VectorXd after_trans = delta.head(3);
    VectorXd after_rot = delta.tail(3);

    Matrix4d new_model = Matrix4d::Identity();
    Sophus::SO3d r = Sophus::SO3d::exp(after_rot);
    new_model.block<3,3>(0,0) = r.matrix();
    new_model.block<3,1>(0,3) = after_trans;

    return new_model;

}

VectorXd pseudo_log( const Matrix4d& transform){

    Matrix3d rot = transform.block<3,3>(0,0);
    VectorXd trans = transform.block<3,1>(0,3);
    Sophus::SO3d rotation(rot);

    // cout << "Current rotation matrix is \n" << rotation.matrix() << " \n" << endl;

    VectorXd rot_delta = rotation.log(); 
    VectorXd delta(6);
    delta << trans, rot_delta;

    return delta;

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
bool SearchNeighbour (const cv::Mat& image, int x, int y ) {
 
    int count = 0;

	for (int i = -1; i < 2; i++) {
		for (int j = -1; j < 2; j++) {            
            // in opencv index of mat is as (row, column), in another words (y,x)
			if ( image.at<cv::Vec3b>(y+i,x+j)[0] < 50 && 
                 image.at<cv::Vec3b>(y+i,x+j)[1] < 50 &&
                 image.at<cv::Vec3b>(y+i,x+j)[2] < 50 &&
                1 < x+j && x+j < image.cols-2 && 0 < y+i && y+i < image.rows-2 &&
                1 < x && x < image.cols-2 && 0 < y && y < image.rows-2 ){ 

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
Eigen::Matrix4d CVGLConversion ( const Eigen::Matrix4d& mat ){

    Eigen::Matrix4d out_mat;
    
    out_mat = mat;
    for (int r = 1; r < 3; r++){
        for (int c = 0; c < 4; c++){
            out_mat(r,c) = -mat(r,c);
        }
    }

    return out_mat;

}

// Project the 3D point on the opencv image with defined coordinate in opengl coordinate
Eigen::Vector2i ProjectOnCVimage (int width, int height, Eigen::Matrix4d perspective, Eigen::Matrix4d camera_pose, Eigen::Matrix4d model_pose, const Eigen::Vector3d& vertex){

    Eigen::Vector4d vertex_hat = pi3to4f(vertex);

    Eigen::Vector4d projection = perspective * camera_pose * model_pose * vertex_hat;
    Eigen::Vector3d x_hat = pi4to3f(projection);
        
    Eigen::Vector2i pixel;    
    pixel[0] = FloatRoundToInt(x_hat[0]*width*0.5+width*0.5);
    pixel[1] = FloatRoundToInt(-x_hat[1]*height*0.5+height*0.5);

    return pixel;
}

// Select 3D point on the silhouette of the image
vector<Vector3d> SelectSilhouettePoint (Mat image, Matrix4d perspective, Matrix4d camera_pose, Matrix4d model_pose, const vector<glm::vec3>& vertex){

    vector<Vector3d> v_silhouette3d;
    int width = image.cols;
    int height = image.rows;
    
    for (int i = 0; i < vertex.size(); ++i){

        Vector3d v = ConvertGlmToEigen3f(vertex[i]);

        Eigen::Vector2i pixel = ProjectOnCVimage(width, height, perspective, camera_pose, model_pose, v);

        if (SearchNeighbour(image, pixel[0], pixel[1]) == 1){

            v_silhouette3d.push_back(v);
        }
    }
    
    return v_silhouette3d;

}

// Compute distance transform of the input, first denoising and then compute distance transform
Mat DistanceMap ( const Mat& original, const Mat& noise ){

    Mat im_temp = original - noise; 
    // Mat im_temp2;
    // imwrite("/home/xinghui/Find-Silhouette/report/no_back_noise.png", im_temp);
    Mat denoise;

    int morph_elem = 0;
    // morph_size defines the size of the kernel
    int morph_size = 2;
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
    morphologyEx(im_temp, im_temp, morph_operator, element);
    // morphologyEx(im_temp, im_temp, morph_operator, element);
    // imwrite("/home/xinghui/Find-Silhouette/report/after_closing.png", im_temp);
    // imshow("denoised input", denoise);
    threshold(im_temp, denoise, 50, 255, cv::THRESH_BINARY);
    // imwrite("/home/xinghui/Find-Silhouette/report/after_binary.png", denoise);
    Mat edge;
    Canny(denoise, edge, 0, 255, 3, true);
    // imwrite("/home/xinghui/Find-Silhouette/report/after_canny.png", edge);
    Mat binary;
    threshold(edge, binary, 50, 255, cv::THRESH_BINARY_INV);
    // imwrite("/home/xinghui/Find-Silhouette/report/after_inv.png", binary);
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
Eigen::MatrixXd dev_dist( const Mat& image, int image_x, int image_y){

    Eigen::MatrixXd gradient (1,2);

    gradient(0,0) = (float) (image.at<float>(image_y, image_x + 1) - image.at<float>(image_y, image_x - 1))/2.f;
    gradient(0,1) = (float) (image.at<float>(image_y + 1, image_x) - image.at<float>(image_y - 1, image_x))/2.f;

    return gradient;

}

Eigen::MatrixXd dev_dist_double( const Mat& image, double image_x, double image_y){

    Eigen::MatrixXd gradient (1,2);

    gradient(0,0) = (double) (bilinearInterpolate<double>(image, image_x+0.1, image_y) - bilinearInterpolate<double>(image, image_x-0.1, image_y))/.2f;
    gradient(0,1) = (double) (bilinearInterpolate<double>(image, image_x, image_y+0.1) - bilinearInterpolate<double>(image, image_x, image_y-0.1))/.2f;

    return gradient;

}


// Compute the jacobian matrix of the pi3to2 function
Eigen::MatrixXd dev_pi3to2(double x, double y, double z){

    Eigen::MatrixXd out_mat(2,3);

    out_mat(0,0) = 1/z;
    out_mat(0,1) = 0;
    out_mat(0,2) = -x/(z*z);
    out_mat(1,0) = 0;
    out_mat(1,1) = 1/z;
    out_mat(1,2) = -y/(z*z);

    return out_mat;
}


double Area (triangle tri){
    
    Vector3d a = tri.vertex1;
    Vector3d b = tri.vertex2;
    Vector3d c = tri.vertex3;

    Vector3d ab = a-b;
    Vector3d ac = a-c;
    
    double theta = acos(ab.dot(ac)/(ab.norm()* ac.norm()));
    
    double area = 0.5 * ab.norm() * ac.norm() * sin(theta);


    return area;
}

vector<int> indexSort( vector<double> x ){

    vector<int> index;

    index.resize(x.size());
    std::size_t n(0);
    std::generate(std::begin(index), std::end(index), [&]{ return n++; });

    std::sort(  std::begin(index), 
                std::end(index),
                [&](int i1, int i2) { return x[i1] < x[i2]; } );

    return index;

}

void LoadModelQuad(std::string Filename, vector<Vector3d> & Vertices, vector< vector<int> >& face)
{
    FILE * Model;

    Model = fopen(Filename.c_str(),"r");

    if( Model == NULL){
        std::cout << "Cannot open the file" <<std::endl;
    }

    std::cout << "Successfully open the file" << std::endl;

    while(1) {
        char lineHeader [80];


        int res = fscanf(Model,"%s", lineHeader);

        if (res == EOF) {
            std::cout << "Complete Reading the file" << std::endl;
            break;
        }

        if (strcmp(lineHeader, "v") == 0) {
            Eigen::Vector3d Vertex;
            float a,b,c;
            fscanf(Model, "%f %f %f\n", &a, &b, &c);
            Vertex[0] = a;
            Vertex[1] = b;
            Vertex[2] = c;
            Vertices.push_back(Vertex);
        } else if(strcmp(lineHeader,"f")==0){
            std::vector<int> vertexIndex (4);
            fscanf(Model, "%d//%*d %d//%*d %d//%*d %d//%*d\n", &vertexIndex[0], &vertexIndex[1], &vertexIndex[2],&vertexIndex[3]);
            face.push_back(vertexIndex);
        }
    }

    std::cout << "Complete Loading the model" << std::endl;

    }

bool c_vertex( const Vector3d& a, const Vector3d& b){

    if (a[0] == b[0] && a[1] == b[1] && a[2] == b[2]){

        return true;
    }else{

        return false;
    }

} 

bool c_edge( const edge& a, const edge& b){
    
    if ( c_vertex(a.vertex1, b.vertex1) || c_vertex(a.vertex1, b.vertex2)){

        if ( c_vertex(a.vertex2, b.vertex1) || c_vertex(a.vertex2, b.vertex2) ){

            return true;
        }
    }else{

        return false;
    }

}

// optimizer calss
// Input: two rotation matrix in opengl frame, one camera intrinsic matrix and 3D silhouette points
// Output: Jacobian matrix, E0 matrix and delta vector

    
optimizer::optimizer(vector<Eigen::Vector3d> v_silhouette3d, Eigen::Matrix3d K, Eigen::Matrix4d camera_pose, Eigen::Matrix4d Model, Mat dist)
: v_silhouette3d(v_silhouette3d), K(K), camera_pose(camera_pose), Model(Model), dist(dist) {
    this->T = CVGLConversion(this->camera_pose * this->Model);
    // cout << std::setprecision(30) << this->T << endl;
}

Eigen::MatrixXd optimizer::GetE0(){
    
    Eigen::MatrixXd E0(this->v_silhouette3d.size(), 1);

    for (int i = 0; i < this->v_silhouette3d.size(); i++){

        Eigen::Vector3d p = this->v_silhouette3d[i];
        Eigen::Vector3d p_hat = pi4to3f(this->T * pi3to4f(p));
        Eigen::Vector2d x = pi3to2f( this->K * p_hat);

        if(false){
            int image_x = FloatRoundToInt(x[0]);
            int image_y = FloatRoundToInt(x[1]);

            float e0 = (float) this->dist.at<float>(image_y, image_x);
            E0(i,0) = e0;    
        }
        else{
            
            double image_x = x[0];
            double image_y = x[1];

            double e0 = bilinearInterpolate<double>(this->dist, image_x, image_y);

            E0(i,0) = e0;    
        }
    }

    return E0;
}



Eigen::MatrixXd optimizer::GetJ(){

    Eigen::MatrixXd J(this->v_silhouette3d.size(), 6);

    for (int i = 0; i < this->v_silhouette3d.size(); i++){

        Eigen::Vector3d p = this->v_silhouette3d[i];
        Eigen::Vector3d p_hat = pi4to3f(this->T * pi3to4f(p));
        Eigen::Vector3d x_hat = this->K * p_hat;
        Eigen::Vector2d x = pi3to2f( x_hat );
        double image_x = x[0];
        double image_y = x[1];

        Eigen::MatrixXd grad_dist;
        if(false)
            grad_dist = dev_dist(this->dist, FloatRoundToInt(image_x), FloatRoundToInt(image_y));
        else
            grad_dist = dev_dist_double(this->dist, image_x, image_y);
        Eigen::MatrixXd grad_pi = dev_pi3to2(x_hat[0], x_hat[1], x_hat[2]);

        Eigen::MatrixXd gpK = grad_dist * grad_pi * this->K;
        Eigen::Vector3d gpKT = gpK.transpose();
        // cout << "-----------------------------------------------------------" << endl;
        // cout << "gradient: " << grad_dist << endl;
        // cout << "-----------------------------------------------------------" << endl;
        // cout << "gpK: " << gpK << endl;
        // cout << "-----------------------------------------------------------" << endl;
        // cout << "p_hat: " << p_hat.transpose() << endl;
        // cout << "-----------------------------------------------------------" << endl;
        Eigen::Vector3d cross = p_hat.cross(gpKT);
        // cout << "cross product: " << cross.transpose() << endl;
        // cout << "-----------------------------------------------------------" << endl;
        Eigen::MatrixXd jacobian(1,6);
        jacobian << gpK, cross.transpose(); 
        // cout << "jacobian: "<< jacobian << endl;
        // cout << "-----------------------------------------------------------" << endl;
        // cout << "E(i,0): " << E0(i,0) << endl;
        // cout << "-----------------------------------------------------------" << endl;

        for (int j = 0; j < 6; j++){
            
            J(i,j) = jacobian(0,j);

        }

    }

    return J;
}

Eigen::MatrixXd optimizer::GetDelta(){

    Eigen::MatrixXd delta(6,1);
    Eigen::MatrixXd E0 = GetE0();
    Eigen::MatrixXd J = GetJ();
    Eigen::MatrixXd temp = J.transpose()*J; 
    delta = temp.inverse()*J.transpose()*E0;

    // cout << "Current delta is " << endl;
    // cout << delta << endl;

    return delta;
}

Eigen::MatrixXd optimizer::GetDev(){

    MatrixXd dev(1,6); 
    dev = MatrixXd::Zero(1,6);

    for (int i = 0; i < this->v_silhouette3d.size(); i++){

        Eigen::Vector3d p = this->v_silhouette3d[i];
        Eigen::Vector3d p_hat = pi4to3f(this->T * pi3to4f(p));
        Eigen::Vector3d x_hat = this->K * p_hat;
        Eigen::Vector2d x = pi3to2f( x_hat );
        int image_x = FloatRoundToInt(x[0]);
        int image_y = FloatRoundToInt(x[1]);

        Eigen::MatrixXd grad_dist = dev_dist(this->dist, image_x, image_y);
        Eigen::MatrixXd grad_pi = dev_pi3to2(x_hat[0], x_hat[1], x_hat[2]);

        Eigen::MatrixXd gpK = grad_dist * grad_pi * this->K;

        MatrixXd T_dev(3,6);
        float X = p[0];
        float Y = p[1];
        float Z = p[2];
        T_dev << 1,0,0,0,4*Z,-4*Y,0,1,0,-4*Z,0,4*X,0,0,1,4*Y,-4*X,0;

        dev += gpK * T_dev;
    }

    return dev;

}

Eigen::Matrix4d optimizer::GetT(){

    return this->T;

}

Eigen::MatrixXd optimizer::LMalgorithm( float lambda ){

    MatrixXd J = GetJ();
    MatrixXd E = GetE0();

    MatrixXd J_squared = J.transpose()*J;
    VectorXd diagonal = J_squared.diagonal();

    MatrixXd J_diagonal = diagonal.asDiagonal();

    MatrixXd delta_lambda = -(J_squared + lambda*J_diagonal).inverse() * (J.transpose() * E);
    
    return delta_lambda;
}

void optimizer::Draw( Mat image ){

    for ( int i = 0; i < this -> v_silhouette3d.size(); i ++){

        Eigen::Vector3d p = this->v_silhouette3d[i];
        Eigen::Vector3d p_hat = pi4to3f(this->T * pi3to4f(p));
        Eigen::Vector3d x_hat = this->K * p_hat;
        Eigen::Vector2d x = pi3to2f( x_hat );
        int image_x = FloatRoundToInt(x[0]);
        int image_y = FloatRoundToInt(x[1]);

        DrawPoint(image, image_x, image_y);

    }

}

VectorXd optimizer::desperate(){

    VectorXd output(6,1); output.setZero();

    VectorXd delta = pseudo_log( this->T );
    // cout << "delta " << delta.tail<3>() << endl;
    // cout << "delta " << delta.tail<3>().norm() << endl;

    float trans_step = 1;
    float rot_step = 0.001;

    for (int i = 0; i < 3; i++){

        VectorXd up = delta;
        VectorXd down = delta;

        up[i] += trans_step;
        down[i] -= trans_step;

        Matrix4d up_matrix = CVGLConversion(pseudo_exp(up));
        Matrix4d down_matrix = CVGLConversion(pseudo_exp(down));

        optimizer opt_up (this->v_silhouette3d, this->K, this->camera_pose, up_matrix, this->dist);
        // cout << setprecision(30) << opt_up.GetT() << endl;
        optimizer opt_down (this->v_silhouette3d, this->K, this->camera_pose, down_matrix, this->dist);
        // cout << setprecision(30) << opt_down.GetT() << endl;

        float dev = (opt_up.GetE0().sum() - opt_down.GetE0().sum())/(2*trans_step*0.1);

        output(i,0)=dev;

    }

    if(true)
    for (int i = 3; i < 6; i++){

        VectorXd increment(6,1); increment.setZero();
        increment(i,0) = rot_step;

        Matrix4d up = pseudo_exp(increment);
        Matrix4d down = pseudo_exp(-increment);

        Matrix4d rot_up = this->T;
        Matrix4d rot_down = this->T;

        rot_up.block<3,3>(0,0) = up.block<3,3>(0,0) * rot_up.block<3,3>(0,0);
        rot_down.block<3,3>(0,0) = down.block<3,3>(0,0) * rot_down.block<3,3>(0,0);

        Matrix4d up_matrix = CVGLConversion(rot_up);
        Matrix4d down_matrix = CVGLConversion(rot_down);

        optimizer opt_up (this->v_silhouette3d, this->K, this->camera_pose, up_matrix, this->dist);
        // cout << setprecision(30) << opt_up.GetT() << endl;
        optimizer opt_down (this->v_silhouette3d, this->K, this->camera_pose, down_matrix, this->dist);
        // cout << setprecision(30) << opt_down.GetT() << endl;
        float dev = (opt_up.GetE0().sum() - opt_down.GetE0().sum())/(2*rot_step*1000000);

        output(i,0)=dev;
    }

    return output;


}

std::vector<Vector3d> optimizer::GetPoint(){

    return this->v_silhouette3d;
}


