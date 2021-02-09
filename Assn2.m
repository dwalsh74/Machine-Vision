
close all
clear all

startTime = now();

im1 = imread('im2_crop.png');
gray1 = rgb2gray(im1);
bwIm1 = im2bw(im1);

sobel1 = edge(gray1,'sobel');
canny1 = edge(gray1,'canny');

% figure;
% imshow(sobel1);

figure;
imshow(canny1);
 
testIm = canny1; 

% width = 440;
% height = 600;
width = 479;
height = 640; 

voting = zeros(width,height,90);
scores = zeros(width,height,2);

%Iterate through all pixels in image

for x = 5:width
    for y = 5:height
        for radius = 15:90
             for theta = 0:2:360 %the possible  theta 0 to 360 

                a = round(x - radius * cos(theta * pi / 180)); %polar coordinate for center
                b = round(y - radius * sin(theta * pi / 180));  %polar coordinate for center 
                
                if a > 0
                    if b > 0
                        if a < width+1
                            if b < height+1
                                if testIm(a,b) > 0
                                    voting(x,y,radius) = voting(x,y,radius) + 1;
                                end
                            end
                        end
                    end
                end
             end 
        end

        %Get circle with highest score

        highScore = 0;
        maxindex = 1;

        for i = 1:90
            if voting(x,y,i) > highScore
                maxindex = i;
                highScore = voting(x,y,i);
            end 
        end

        scores(x,y,1) = highScore;
        scores(x,y,2) = maxindex;

    end
end

%binarize image based on score threshold

bwIm = scores(:,:,1) > 90;

figure;
imagesc(scores(:,:,1));
figure;
imshow(bwIm);

executionTime = now() - startTime;
fprintf("Execution Time: %f",executionTime);

 