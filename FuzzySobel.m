% Khamy fuzzy sobel operator
%
% Khamy, E.L. et al., Modified Sobel fuzzy edge
% detector, in Proceedings of 17th National Radio
% Science Conference (NRSC 2000), C32-1-9,
% Minufi a, Egypt, 2000.
%
% coded in MAtlab by kambiz.rahbar@gmail.com - Jan 2019
%

clc
clear
close all

% load a image
img = imread('test1.jpg');

% make image grayscale if necessary
if size(img, 3)>1
    img = rgb2gray(img);
end

% convert image class from uint8 to double
img = double(img);

% show the original image
figure(1);
subplot(2,2,1);
imshow(uint8(img));
title('original image');

%% sobel operator
% define Sobel operators
Sobel_x = [-1 0 1;
    -2 0 2;
    -1 0 1];

Sobel_y = [-1 -2 -1;
    0   0  0;
    1   2  1];

% apply sobel operators on image
Gx = conv2(img, Sobel_x, 'same');
Gy = conv2(img, Sobel_y, 'same');

% calculate the magnitude
G = sqrt(Gx.^2+Gy.^2);

% normalized magnitude in [0:100]
G = G / max(G(:)) * 100;

% set a threshold and apply it
% threshold find by try and error
sobel_threshold = 10;
sobel_edges = G > sobel_threshold;

% show the results from original sobel operators
subplot(2,2,2);
imshow(sobel_edges);
title('edge using sobel operator');

%% Khamy fuzzy sobel operator
diff = zeros(size(img));
neighber_diff = zeros(3);

% calculate diff matrix according to Khamy paper
for x = 2:size(img, 1)-1
    for y = 2:size(img, 2)-1
        
        m = 0;
        for i = -1:1
            m = m+1;
            
            n = 0;
            for j = -1:1
                n = n+1;
                neighber_diff(m,n) = abs(img(x,y)-img(x+i, y+j));
            end
        end
        
        diff(x,y) = max(neighber_diff(:));
    end
end

% calculate diff histogram and plot it
diff_hist = hist(diff(:),255);
subplot(2,2,3);
bar(diff_hist);
title('diff histogram');

% calculate the 20% of the maximum peak of the diff histogram
peak0_02 = 20/100 * max(diff_hist);

% estimate the low_threshold (LT)
for i = 1:255
    if diff_hist(i) >= peak0_02
        low_threshold = i;
        break;
    end
end

% estimate the high_threshold (HT)
for i = 255:-1:0
    if diff_hist(i) >= peak0_02
        high_threshold = i;
        break;
    end
end

% calculate high_limit (HL) and low_limit (LL)
high_limit = low_threshold + (high_threshold - low_threshold)/3;
low_limit = high_threshold - (high_threshold - low_threshold)/3;

% calculate mu_SFR matrix according to Khamy paper
mu_SFR = 1/(low_threshold-low_limit)*(G-low_limit);
mu_SFR(G>=low_limit) = 0;
mu_SFR(G<=low_threshold) = 1;

% calculate mu_EFR matrix according to Khamy paper
mu_EFR = 1/(high_threshold-high_limit)*(G-high_limit);
mu_EFR(G<=high_limit) = 0;
mu_EFR(G>=high_threshold) = 1;

% calculate the edges of image and store them into R matrix
R = zeros(size(img));
for x=1:size(G,1)
    for y=1:size(G,2)
        if G (x, y) >= high_threshold
            R(x,y) = 255;
        elseif G (x, y) <= low_threshold
            R(x,y) = 0;
        else
            R(x,y) = G(x,y)*max(mu_SFR(x,y), mu_EFR(x,y));
        end
    end
end

% show the results from Khamy fuzzy sobel operator
subplot(2,2,4);
imshow(uint8(R));
title('edge using fuzzy sobel operator');
