function r=practical2

%We assume that the intrinsic camera matrix K is known and has values
K = [640  0    320;...
     0    640  240;
     0    0    1];
 
%We will assume an object co-ordinate system with the Z-axis pointing upwards and the
%origin in the centre of the plane. There are four known points on the plane, with coordinates
%(in mm):

XCart = [-100 -100  100  100 0 ;...
         -100  100  100 -100 0;...
          0    0    0    0   0 ];

%We will assume that the correct transformation from the plane co-ordinate system to the
%camera co-ordinate system (extrinsic matrix) is:

T = [ 0.9851  -0.0492  0.1619  46.00;...
     -0.1623  -0.5520  0.8181  70.00;...
      0.0490  -0.8324 -0.5518  500.89;...
      0        0       0       1]
  
% TO DO  Use the general pin-hole projective camera model discussed in the lectures to estimate 
%where the four points on the plane will appear in the image.  Fill in the
%details of the function "projectiveCamera" - body of function appears below

xImCart = projectiveCamera(K,T,XCart);

% TO DO Add noise to the pixel positions to simulate having to find these points in a noisy
%image. Store the results back in xImCart.  
%The noise should have standard deviation of one pixel in each direction.
xImCart = xImCart+randn(size(xImCart));

%Now we will take the image points and the known positions on the card and try to
%estimate the extrinsic matrix using the algorithm discussed in the lecture. 
%Fill in the details of the function "estimate plane pose" - body of function appears
%below

TEst = estimatePlanePose(xImCart,XCart,K)

%if you have got this correct, it should resemble T above.

%==========================================================================
%==========================================================================

%goal of function is to project points in XCart through projective camera
%defined by intrinsic matrix K and extrinsic matrix T.
function xImCart = projectiveCamera(K,T,XCart);
%TO DO convert Cartesian 3d points XCart to homogeneous coordinates XHom
xHom = [XCart; ones(1,size(XCart,2))];
%TO DO apply extrinsic matrix to XHom to move to frame of reference of
%camera
ref_cam = T*xHom;
%TO DO project points into normalized camera coordinates xCamHom by (achieved by
%removing fourth row)
xCamHom = ref_cam(1:3,:);
%TO DO move points to image coordinates xImHom by applying intrinsic matrix
xImHom = K*xCamHom;
%TO DO convert points back to Cartesian coordinates xImCart
xImCart = xImHom(1:2,:)./repmat(xImHom(3,:),2,1);


%==========================================================================
%==========================================================================

%goal of function is to estimate pose of plane relative to camera
%(extrinsic matrix) given points in image xImCart, points in world XCart
%and intrinsic matrix K.

function T = estimatePlanePose(xImCart,XCart,K)

%TO DO Convert Cartesian image points xImCart to homogeneous representation
%xImHom
xImHom = [xImCart; ones(1,size(xImCart,2))];
%TO DO Convert image coordinates xImHom to normalized camera coordinates
%xCamHom
xCamHom = K\xImHom;
%TO DO Estimate homography H mapping homogeneous (x,y)
%coordinates of positions in real world to xCamHom.  Use the routine you wrote for
%Practical 1B.
HEst_real_cam = calcBestHomography(XCart(1:2,:), xCamHom(1:2,:));
%TO DO Estimate first two columns of rotation matrix R from the first two
%columns of H using the SVD
[U,~,V] = svd(HEst_real_cam(:,1:2));
R_1_2 = U*[1,0;0,1;0,0]*V';
%TO DO Estimate the third column of the rotation matrix by taking the cross
%product of the first two columns
R_3 = cross(R_1_2(:,1),R_1_2(:,2));
%TO DO Check that the determinant of the rotation matrix is positive - if
%not then multiply last column by -1.
R = [R_1_2 , R_3];
if (det(R) < 0)
    R = R.*repmat([1,1,-1],3,1);
end    
%TO DO Estimate the translation t by finding the appropriate scaling factor k
%and applying it to the third colulmn of H
k = sum((sum((HEst_real_cam(:,1:2)./R_1_2)')))/6;
t = HEst_real_cam(:,3)/k;
%TO DO Check whether t_z is negative - if it is then multiply t by -1 and
%the first two columns of R by -1.
if (t(3) < 0)
    t = t*-1;
    R = R.*repmat([-1,-1,1],3,1);
end
%assemble transformation into matrix form
T  = [R t;0 0 0 1];


%==========================================================================
function H = calcBestHomography(pts1Cart, pts2Cart)

%should apply direct linear transform (DLT) algorithm to calculate best
%homography that maps the points in pts1Cart to their corresonding matching in 
%pts2Cart
%****TO DO ****: replace this
[ndim,npoints] = size(pts1Cart);
%one_vector = ones(1,npoints);
%**** TO DO ****;
%first turn points to homogeneous
%pts1Cart_hom = [pts1Cart; one_vector]
%pts2Cart_hom = [pts2Cart; one_vector]
%then construct A matrix which should be (10 x 9) in size
%solve Ah = 0 by calling
%h = solveAXEqualsZero(A); (you have to write this routine too - see below)
A = zeros(10,9);

for (i=1:npoints)
    A(2*i-1,:) = [0,0,0,pts1Cart(1,i),pts1Cart(2,i),1,-pts2Cart(2,i)*pts1Cart(1,i),-pts2Cart(2,i)*pts1Cart(2,i),-pts2Cart(2,i)];
    A(2*i,:) = [pts1Cart(1,i),pts1Cart(2,i),1,0,0,0,-pts2Cart(1,i)*pts1Cart(1,i),-pts2Cart(1,i)*pts1Cart(2,i),-pts2Cart(1,i)];
end
h = solveAXEqualsZero(A);
H = reshape(h,3,3)';


%==========================================================================
function x = solveAXEqualsZero(A);

[~,~,V] = svd(A);
x = V(:,size(A,2));
%****TO DO **** Write this routine 

