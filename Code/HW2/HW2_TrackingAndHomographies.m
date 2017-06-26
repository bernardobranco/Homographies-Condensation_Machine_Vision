function HW2_TrackingAndHomographies


LLs = HW2_Practical9c( 'll' );
LRs = HW2_Practical9c( 'lr' );
ULs = HW2_Practical9c( 'ul' );
URs = HW2_Practical9c( 'ur' );

close all;

% Load frames from the whole video into Imgs{}.
% This is really wasteful of memory, but makes subsequent rendering faster.
LoadVideoFrames

% Coordinates of the known target object (a dark square on a plane) in 3D:
XCart = [-50 -50  50  50;...
          50 -50 -50  50;...
           0   0   0   0];

% These are some approximate intrinsics for this footage.
K = [640  0    320;...
     0    512  256;
     0    0    1];

% Define 3D points of wireframe object.
XWireFrameCart = [-50 -50  50  50 -50 -50  50  50;...
                   50 -50 -50  50  50 -50 -50  50;...
                    0   0   0   0 -100 -100 -100 -100];
 
hImg = figure;
       
% ================================================
for iFrame = 1:numFrames
    xImCart = [LLs(iFrame,:)' ULs(iFrame,:)' URs(iFrame,:)' LRs(iFrame,:)']
    xImCart = circshift( xImCart, 1)

    % To get a frame from footage 
    im = Imgs{iFrame};

    % Draw image and 2d points
    set(0,'CurrentFigure',hImg);
    set(gcf,'Color',[1 1 1]);
    imshow(im); axis off; axis image; hold on;
    plot(xImCart(1,:),xImCart(2,:),'r.','MarkerSize',15);


    %TO DO Use your routine to calculate TEst the extrinsic matrix relating the
    %plane position to the camera position.
    T = estimatePlanePose(xImCart, XCart, K);

    %TO DO Draw a wire frame cube, by projecting the vertices of a 3D cube
    %through the projective camera, and drawing lines betweeen the 
    %resulting 2d image points
    
    
    hold on;
    
    % TO DO: Draw a wire frame cube using data XWireFrameCart. You need to
    % 1) project the vertices of a 3D cube through the projective camera;
    xImCart_cube = projectiveCamera(K,T,XWireFrameCart);
    % 2) draw lines betweeen the resulting 2d image points.
    % Note: CONDUCT YOUR CODE FOR DRAWING XWireFrameCart HERE
    plotCube(xImCart_cube)
    
    hold off;
    drawnow;
    
%     Optional code to save out figure
%     pngFileName = sprintf( '%s_%.5d.png', 'myOutput', iFrame );
%     print( gcf, '-dpng', '-r80', pngFileName ); % Gives 640x480 (small) figure

    
end % End of loop over all frames.
% ================================================

% TO DO: QUESTIONS TO THINK ABOUT...

% Q: Do the results look realistic?
% If not then what factors do you think might be causing this


% TO DO: your routines for computing a homography and extracting a 
% valid rotation and translation GO HERE. Tips:
%
% - you may define functions for T and H matrices respectively.
% - you may need to turn the points into homogeneous form before any other
% computation. 
% - you may need to solve a linear system in Ah = 0 form. Write your own
% routines or using the MATLAB builtin function 'svd'. 
% - you may apply the direct linear transform (DLT) algorithm to recover the
% best homography H.
% - you may explain what & why you did in the report.


%==========================================================================
%==========================================================================

%==========================================================================
%==========================================================================

%goal of function is to project points in XCart through projective camera
%defined by intrinsic matrix K and extrinsic matrix T.
function xImCart = projectiveCamera(K,T,XCart);
%replace this
xImCart = [];
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
[U,L,V] = svd(HEst_real_cam(:,1:2));
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



%==========================================================================
function plotCube(cube_points)
sq1 = [cube_points(:,1:4) cube_points(:,1)]
sq2 = [cube_points(:,5:8) cube_points(:,5)]
line1 = [cube_points(:,1) cube_points(:,5)]
line2 = [cube_points(:,2) cube_points(:,6)]
line3 = [cube_points(:,3) cube_points(:,7)]
line4 = [cube_points(:,4) cube_points(:,8)]

plot([sq1(1,:)],[sq1(2,:)],'Color','g','LineWidth',1)
hold on;
plot([sq2(1,:)],[sq2(2,:)],'Color','g','LineWidth',1)
hold on;
plot([line1(1,:)],[line1(2,:)],'Color','g','LineWidth',1)
hold on;
plot([line2(1,:)],[line2(2,:)],'Color','g','LineWidth',1)
hold on;
plot([line3(1,:)],[line3(2,:)],'Color','g','LineWidth',1)
hold on;
plot([line4(1,:)],[line4(2,:)],'Color','g','LineWidth',1)
hold on;





