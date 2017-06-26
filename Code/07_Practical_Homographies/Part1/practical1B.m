function practical1B

%close all open figures
close all;

%load in the required data
load('PracticalData','im1','im2','im3','pts1','pts2','pts3','pts1b');
%im1 is center image with grey background
%im2 is left image 
%pts1 and pts2 are matching points between image1 and image2
%im3 is right image
%pts1b and pts3 are matching points between image 1 and image 3

%show images and points
figure; set(gcf,'Color',[1 1 1]);image(uint8(im1));axis off;hold on;axis image;
plot(pts1(1,:),pts1(2,:),'r.'); 
plot(pts1b(1,:),pts1b(2,:),'m.');
figure; set(gcf,'Color',[1 1 1]);image(uint8(im2));axis off;hold on;axis image;
plot(pts2(1,:),pts2(2,:),'r.'); 
figure; set(gcf,'Color',[1 1 1]);image(uint8(im3));axis off;hold on;axis image;
plot(pts3(1,:),pts3(2,:),'m.'); 



%****TO DO**** 
%for every pixel in image 1
    %transform this pixel position with your homography to find where it 
    %is in the coordinates of image 2
    %if it the transformed position is within the boundary of image 2 then 
        %copy pixel colour from image 2 pixel to current position in image 1 
        %draw new image1 (use drawnow to force it to draw)
    %end
%end;

[r,c,~] = size(im1);

%****TO DO**** 
%calculate homography from pts1 to pts2
HEst_1_2 = calcBestHomography(pts1, pts2);

for (y=1:r)
    for (x=1:c)
        % turn pixel to homogeneous representation
        %apply homography to points
        % x,y,1 because thats how we apply the homography matrix:
        pts1Hom = [x;y;1];
        pts2EstHom = HEst_1_2*pts1Hom;
        % see if transformed position is within the boundaries of the
        % target image
        pts2EstCart = pts2EstHom(1:2,:)./repmat(pts2EstHom(3,:),2,1);
        if (pts2EstCart(2) <= size(im2,1) && pts2EstCart(2) > 1) && (pts2EstCart(1) <= size(im2,2) && pts2EstCart(1) > 1)
            %copy pixel colour from image 2 pixel to current position in image 1
            % y,x because thats how we plot the points in our pic, first
            % row (y) then column (x)
            im1(y,x,:) = im2(floor(pts2EstCart(2)),floor(pts2EstCart(1)),:);        
        end
    end
end

HEst_1_3 = calcBestHomography(pts1b, pts3);
%****TO DO****
%repeat the above process mapping image 3 to image 1.
for (y=1:r)
    for (x=1:c)
        % turn pixel to homogeneous representation
        %pts1Hom = [im1(i,j,1); im1(i,j,2); im1(i,j,3); 1];
        %apply homography to points
        pts1Hom = [x;y;1];
        pts3EstHom = HEst_1_3*pts1Hom;
        % see if transformed position is within the boundaries of the
        % target image
        %pts2EstHom_Nom = pts2EstHom(1:2) / pts2EstHom(3);
        pts3EstCart = pts3EstHom(1:2,:)./repmat(pts3EstHom(3,:),2,1);
        pts3EstCart_floor = [floor(pts3EstCart(1)),floor(pts3EstCart(2))];
        if (pts3EstCart_floor(2) <= size(im3,1) && pts3EstCart_floor(2) > 0) && (pts3EstCart_floor(1) <= size(im3,2) && pts3EstCart_floor(1) > 0)
            %copy pixel colour from image 2 pixel to current position in image 1
            im1(y,x,:) = im3(pts3EstCart_floor(2),pts3EstCart_floor(1),:);
        end
    end
end
figure; set(gcf,'Color',[1 1 1]);image(uint8(im1));axis off;hold on;axis image;


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


