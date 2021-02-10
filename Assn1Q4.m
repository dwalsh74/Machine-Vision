%Cleanup
clear all
close all
clearvars

%Read image to MATLAB
Im = imread('Lenna.png');

%Convert to Grayscale Image
Im = rgb2gray(Im);
Im = im2double(Im);
whos('I')

%Display Grayscale Image
figure;
imshow(Im);
title('Grayscale Image');

%% Canny Edge Detection %%

%Constants
filterSize = 5;
sigma = 14;

%Create Gaussian Filter
filter = fspecial('gaussian', filterSize, sigma);

[h, w] = size(Im);

%Sobel Mask
Mx = [-1 0 1; -2 0 2; -1 0 1];
My = [-1 -2 -1; 0 0 0; 1 2 1];

%Apply Gaussian Filter
for i = 2 : 511
    
    for j = 2 : 511
        
        sum = 0;
        
        for i1 = -2 : 2
            
            for j1 = -2 : 2
                sum = sum + (Im(j, i) * filter(i1+3, j1+3));
            end    
        end
        
        Imx1(j,i) = sum;
        
    end
end

%Apply sobel mask to x direction
for i = 2:510

         for j = 2:510
             
             sum1 = 0;
            
              for i1=-1:1
                  
                for j1 = -1:1
                    sum1 = sum1 + Imx1(j + i1, i + j1)* Mx(i1 + 2,j1 + 2);
                end
                
              end
            
            Imx2(j,i) = sum1;
            
         end
end

%Apply sobel mask to y direction
for i = 2:510
    
         for j = 2:510
             
            sum2 = 0;

            for i1=-1:1 
                for j1=-1:1
                    sum2 = sum2 + Imx1(j + i1, i + j1)* My(i1 + 2,j1 + 2);
                end  
            end
            
            Imy2(j,i) = sum2;
            
         end
end


for i = 2:510
    
    for j = 2:510
        mod(j,i) = abs(Imx2(j,i)) + abs(Imy2(j,i));
    end
   
end

%Non-maximum Suppression
suppressedIm(j,i) = 0;
for i = 2 : 509
    for j = 2 : 509
        theta(j,i) = abs(atand(Imy2(j,i)/Imx2(j,i))); 
        
        if ((theta(j,i) >= 0) && (theta(j,i) <= 22.5) || (theta(j,i) >= 157.5) && (theta(j,i) <= 180))
            ntheta(j,i) = 0;
        end
        
        if ((theta(j,i) >= 22.5) && (theta(j,i) <= 67.5))
            ntheta(j,i) = 45;
        end
        
        if ((theta(j,i) >= 67.5) && (theta(j,i) <= 112.5))
            ntheta(j,i) = 90;
        end
        
        if ((theta(j,i) >= 112.5) && (theta(j,i) <= 180))
            ntheta(j,i) = 135;
        end
                       
        if (ntheta(j,i) == 0)
            
            if (mod(j, i) < mod(j-1, i) || mod(j, i) < mod(j+1, i))
                suppressedIm(j,i) = 0;

            else suppressedIm(j,i)= mod(j,i);

            end
        end
                
            
            if (ntheta(j,i) == 45)
                
                if (mod(j, i) < mod(j+1, i-1) || mod(j, i) < mod(j-1, i+1))
                    suppressedIm(j,i) = 0;
                    
                else suppressedIm(j,i)= mod(j,i);
                    
                end
            end
            
            if (ntheta(j,i) == 90)
                
                if (mod(j, i) < mod(j, i-1) || mod(j, i) < mod(j, i+1))
                    suppressedIm(j,i) = 0;
                    
                else suppressedIm(j,i)= mod(j,i);
                    
                end
            end
               
            if (ntheta(j,i) == 135)
                
                if (mod(j, i) < mod(j-1, i-1) || mod(j, i) < mod(j+1, i+1))
                    suppressedIm(j,i) = 0;
                    
                else suppressedIm(j,i) = mod(j,i);
                    
                end
            end
    end
end

%Apply Thresholding
highThresh = 0.35;
lowThresh = 0.004;

outputIm = zeros(512,512);

for i = 2 : 509
    
    for j = 2 : 509
        
        if(suppressedIm(j,i) >= highThresh)
            high(j,i) = suppressedIm(j,i);
            outputIm(j,i) = 1;
        end
        
    end
end
   
%subplot(2,2,1);
figure;
imshow(Im);
title('Original Image');

%subplot(2,2,2);
figure;
imshow(suppressedIm);
title('Suppressed Image');

%subplot(2,2,3);
figure;
imshow(outputIm);
title('Canny Edge Detection Applied Manually');

%Apply Canny Edge Detection Using Built-In Function
BinaryIm = edge(Im,'Canny');

%Display Binary Image
%subplot(2,2,4);
figure;
imshow(BinaryIm);
title('Canny Edge Detection Applied with Function');