% Load images.
%buildingDir = fullfile(toolboxdir('vision'),'visiondata','building');
buildingDir = 'path';
buildingScene = imageDatastore(buildingDir);

% Display images to be stitched.
montage(buildingScene.Files)

% Read the first image from the image set.
I = readimage(buildingScene,1);

%I = imrotate(I,-90);
%M = [false,true,true,false;
%     false,true,true,false;
%     true,true,true,true;
%     true,true,true,true;
%     0,1,1,0;0,1,1,0];

M = [0,1,0;1,1,1;0,1,0];

% Initialize features for I(1)
grayImage = im2gray(I);
gray_img = grayImage;
I_2 = convolve2(grayImage,M,'same');
[l,n] = size(I_2);
I_3 = zeros(l,n);
for i=1:l
    for j=1:n
        if(I_2(i,j)>100)I_3(i,j) = true;
        else I_3(i,j) = false;
        end
    end
end
figure();
imshow(I_3);
[y,x,~] = harris(grayImage,2700);
points = [x,y];
figure();
imshow(gray_img);
hold on;
scatter(x,y);
%points = detectSURFFeatures(grayImage);
[features, points] = extractFeatures(grayImage,points);

% Initialize all the transformations to the identity matrix. Note that the
% projective transformation is used here because the building images are fairly
% close to the camera. For scenes captured from a further distance, you can use
% affine transformations.
numImages = numel(buildingScene.Files);
tforms(numImages) = projtform2d;

% Initialize variable to hold image sizes.
imageSize = zeros(numImages,2);

% Iterate over remaining image pairs
for n = 2:numImages
    % Store points and features for I(n-1).
    pointsPrevious = points;
    featuresPrevious = features;
        
    % Read I(n).
    I = readimage(buildingScene, n);
    %I = imrotate(I,-90);
    
    % Convert image to grayscale.
    grayImage = im2gray(I);    
    
    % Save image size.
    imageSize(n,:) = size(grayImage);
    
    % Detect and extract SURF features for I(n).
    [y,x,~] = harris(grayImage,2700);
    points = [x,y];
    %points = detectSURFFeatures(grayImage);
    [features, points] = extractFeatures(grayImage, points);
  
    % Find correspondences between I(n) and I(n-1).
    indexPairs = matchFeatures(features, featuresPrevious, 'Unique', true);
       
    matchedPoints = points(indexPairs(:,1), :);
    matchedPointsPrev = pointsPrevious(indexPairs(:,2), :);        
    
    % Estimate the transformation between I(n) and I(n-1).
    tforms(n) = estgeotform2d(matchedPoints, matchedPointsPrev,...
        'projective', 'Confidence', 99.9, 'MaxNumTrials', 2000);
    
    % Compute T(1) * T(2) * ... * T(n-1) * T(n).
    tforms(n).A = tforms(n-1).A * tforms(n).A; 
end

%% 

% Compute the output limits for each transformation.
for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(i,2)], [1 imageSize(i,1)]);    
end

avgXLim = mean(xlim, 2);
[~,idx] = sort(avgXLim);
centerIdx = floor((numel(tforms)+1)/2);
centerImageIdx = idx(centerIdx);

Tinv = invert(tforms(centerImageIdx));
for i = 1:numel(tforms)    
    tforms(i).A = Tinv.A * tforms(i).A;
end

%%

for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(i,2)], [1 imageSize(i,1)]);
end

maxImageSize = max(imageSize);

% Find the minimum and maximum output limits. 
xMin = min([1; xlim(:)]);
xMax = max([maxImageSize(2); xlim(:)]);

yMin = min([1; ylim(:)]);
yMax = max([maxImageSize(1); ylim(:)]);

% Width and height of panorama.
width  = round(xMax - xMin);
height = round(yMax - yMin);

% Initialize the "empty" panorama.
panorama = zeros([height width 3], 'like', I);

%%

blender = vision.AlphaBlender('Operation', 'Binary mask', ...
    'MaskSource', 'Input port');  

% Create a 2-D spatial reference object defining the size of the panorama.
xLimits = [xMin xMax];
yLimits = [yMin yMax];
panoramaView = imref2d([height width], xLimits, yLimits);

% Create the panorama.
for i = 1:numImages
    
    I = readimage(buildingScene, i);   
   
    % Transform I into the panorama.
    warpedImage = imwarp(I, tforms(i), 'OutputView', panoramaView);
                  
    % Generate a binary mask.    
    mask = imwarp(true(size(I,1),size(I,2)), tforms(i), 'OutputView', panoramaView);
    
    % Overlay the warpedImage onto the panorama.
    panorama = step(blender, panorama, warpedImage, mask);
end

figure
imshow(panorama)