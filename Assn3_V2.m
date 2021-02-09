
%% ENGI 8814 Assignment 3 Code %%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Submitted by                  %
%                               %
% Ahmed Quadri - 201420783      %
% Amy Barrett  - 201337474      %
% David Walsh  - 201435609      %
%                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Cleanup workspace before running

clear all;
close all;
clc;

%% Step 1 - Load both images, convert to double and to gray scale

im1 = imread('im1.jpg');
im1Double = im2double(im1);
im1Gray = rgb2gray(im1Double);
imageSize1 = size(im1Gray);

im2 = imread('im2.jpg');
im2Double = im2double(im2);
im2Gray = rgb2gray(im2Double);
imageSize2 = size(im2Gray);

%Get max x and y dimensions between the images
maximageSize_y = max(imageSize1(1), imageSize2(1));
maximageSize_x = max(imageSize1(2), imageSize2(2));

%% Step 2 - Detect feature points in both images. You can use Harris corner detector

points1 = detectHarrisFeatures(im1Gray);
points2 = detectHarrisFeatures(im2Gray);

%% Step 3 - Compute descriptors around every keypoint in both images

[features1,valid_points1] = extractFeatures(im1Gray,points1);
[features2,valid_points2] = extractFeatures(im2Gray,points2);

%% Step 4 - Compute matches between every descriptor in one image and every descriptor in the other image. Then select the best matches

indexPairs = matchFeatures(features1,features2);

matchedPoints1 = valid_points1(indexPairs(:,1),:);
matchedPoints2 = valid_points2(indexPairs(:,2),:);


%% Step 5 Finding Transform

tform = estimateGeometricTransform(matchedPoints1, matchedPoints2,'projective', 'Confidence', 99.9, 'MaxNumTrials', 2000);

% Computing output limits
[xlim, ylim] = outputLimits(tform, [1 imageSize1(1,2)], [1 imageSize2(1,1)]);  

% Find the minimum and maximum output limits 
xMin = min([1; xlim(:)]);
xMax = max([maximageSize_x; xlim(:)]);

yMin = min([1; ylim(:)]);
yMax = max([maximageSize_y; ylim(:)]);

% Width and height of panorama
width  = round(xMax - xMin);
height = round(yMax - yMin);

% Initialize the "empty" panorama
panorama = zeros([height width 3], 'like', im1);

xLimits = [xMin xMax];
yLimits = [yMin yMax];

panoramaView = imref2d([height width], xLimits, yLimits);
panorama1 = imwarp(im1, tform, 'OutputView', panoramaView);

% Create stiched together panorama image
tform = affine2d(eye(3));
panorama2 = imwarp(im2, tform, 'OutputView', panoramaView);

panorama = imfuse(panorama1, panorama2, 'blend', 'Scaling', 'joint');

% Convert to grayscale
panorama = rgb2gray(panorama);

%% Display Output Images

figure; 
imshowpair(panorama1,panorama2,'montage');
title('Original Input Images');

figure; 
imshowpair(im1Gray,im2Gray,'montage');
title('Original Input Images in Grayscale');

figure; ax = axes;
showMatchedFeatures(im1Gray,im2Gray,matchedPoints1,matchedPoints2,'montage','Parent',ax);
title(ax, 'Candidate point matches');
legend(ax, 'Matched points 1','Matched points 2');

figure;
imshow(panorama);
title('Stitched Panorama');

