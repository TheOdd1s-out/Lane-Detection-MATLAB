clc;
close all;
clear;

%Region of Interest Coordinates
c = [673 565 513 462 437 428 430 431 431 441 461 478 515 541 581 600 645 657];
r = [285 418 488 538 544 569 612 646 667 693 733 806 901 977 1058 1095 1146 1130];
img_count = 251;
%Output_Video=VideoWriter('Jason Video','MPEG-4');
%Output_Video.FrameRate = 25;
%open(Output_Video);
for i=1:img_count
    % images is the folder name.
    img_path = sprintf('images_update\\out%d.jpg',i);
    img = imread(img_path);
    

    %imshow(img);
    
    img = imgaussfilt3(img); %Doing this to average out the frame to prevent from crashing to load
    
    %imshow(img);

    %% Masking the image for White and Yellow Color

    %Define Thresholds for masking Yellow Color
    %Taken this code from Source Code from MATLAB website
    %Define thresholds for 'Hue'
    channel1MinY = 130;
    channel1MaxY = 255;
    %Define thresholds for 'Saturation'
    channel2MinY = 130;
    channel2MaxY = 255;
    %Define thresholds for 'Value'
    channel3MinY = 0;
    channel3MaxY = 130;

    %Create mask based on chosen histogram thresholds
    Yellow=((img(:,:,1)>=channel1MinY)|(img(:,:,1)<=channel1MaxY))& ...
        (img(:,:,2)>=channel2MinY)&(img(:,:,2)<=channel2MaxY)&...
        (img(:,:,3)>=channel3MinY)&(img(:,:,3)<=channel3MaxY);

    %imshow(Yellow);

    %Define Thresholds for masking White Color

    %Define thresholds for 'Hue'
    channel1MinW = 200;
    channel1MaxW = 255;
    %Define thresholds for 'Saturation'
    channel2MinW = 200;
    channel2MaxW = 255;
    %Define thresholds for 'Value'
    channel3MinW = 200;
    channel3MaxW = 255;

    %Create mask based on chosen histogram thresholds
    White=((img(:,:,1)>=channel1MinW)|(img(:,:,1)<=channel1MaxW))&...
        (img(:,:,2)>=channel2MinW)&(img(:,:,2)<=channel2MaxW)& ...
        (img(:,:,3)>=channel3MinW)&(img(:,:,3)<=channel3MaxW);

    %imshow(White);
    newimg = imfuse(Yellow,White);
    newimg = rgb2gray(newimg);
    %imshow(newimg);

    %% Detecting edges in the image using Canny edge detecter function

    frameW = edge(White, 'canny', 0.2);
    frameY = edge(Yellow, 'canny', 0.2); 

    %imshow(frameW);
    %imshow(frameY);
    %% Deciding ROI Points and Extracting ROI
    %Extracting Region of Interest from Yellow Edge Frame

    roiY = roipoly(frameY, r, c);
    [R , C] = size(roiY);
    for i = 1:R
        for j = 1:C
            if roiY(i,j) == 1
                frame_roiY(i,j) = frameY(i,j);
            else
                frame_roiY(i,j) = 0;
            end
        end
    end
    
    %imshow(frame_roiY);
    

    %Extracting Region of Interest from White Edge Frame

    roiW = roipoly(frameW, r, c);
    [R , C] = size(roiW);
    for i = 1:R
        for j = 1:C
            if roiW(i,j) == 1
                frame_roiW(i,j) = frameW(i,j);
            else
                frame_roiW(i,j) = 0;
            end
        end
    end
    %imshow(frame_roiW);
    newroi = frame_roiW + frame_roiY;
    %imshow(newroi);
% 
    %% Applying Hough Tansform to get straight lines from Image
    
    %Applying Hough Transform to White and Yellow Frames
    
    [H_Y,theta_Y,rho_Y] = hough(frame_roiY);
    [H_W,theta_W,rho_W] = hough(frame_roiW);
    
    %Extracting Hough Peaks from Hough Transform of frames
    
    P_Y = houghpeaks(H_Y,2,'threshold',2);
    P_W = houghpeaks(H_W,2,'threshold',2);

    %Hough Transform plotting lines for the Yellow Lanes
      lines_Y = houghlines(frame_roiY,theta_Y,rho_Y,P_Y,'FillGap',3000,'MinLength',20);
%     imshow(img)
%     hold on
%     for k = 1:length(lines_Y)
%         xy = [lines_Y(k).point1; lines_Y(k).point2];
%         plot(xy(:,1),xy(:,2),'LineWidth',4,'Color','Yellow');
%     end
% 
%     %Hough Transform plotting lines for the White Lanes
     lines_W = houghlines(frame_roiW,theta_W,rho_W,P_W,'FillGap',3000,'MinLength',20);
%     for k = 1:2
%         xy = [lines_W(k).point1; lines_W(k).point2];
%         plot(xy(:,1),xy(:,2),'LineWidth',4,'Color','Blue');
%     end
%     hold off
    
   %% Plotting best fitting line after Extrapolation 
    
    %Extract start and end points of lines
    
    leftp1 = [lines_Y(1).point1; lines_Y(1).point2];
    leftp2 = [lines_Y(2).point1; lines_Y(2).point2];  
    
    rightp1 = [lines_W(1).point1; lines_W(1).point2];
    rightp2 = [lines_W(2).point1; lines_W(2).point2];
    
    if leftp1(1,1) < leftp2(1,1)
        left_plot(1,:) = leftp1(1,:);
    else
        left_plot(1,:) = leftp2(1,:);
    end
    
    if leftp1(2,2) < leftp2(2,2)
        left_plot(2,:) = leftp1(2,:);
    else
        left_plot(2,:) = leftp2(2,:);
    end
    
    if rightp1(1,2) < rightp2(1,2)
        right_plot(1,:) = rightp1(1,:);
    else
        right_plot(1,:) = rightp2(1,:);
    end
    
    if rightp1(2,1) > rightp2(2,2)
        right_plot(2,:) = rightp1(2,:);
    else
        right_plot(2,:) = rightp2(2,:);
    end
    

    %Calculate slope of left and right lines
    
    slopeL = (left_plot(2,2)-left_plot(1,2))/(left_plot(2,1)-left_plot(1,1));
    slopeR = (right_plot(2,2)-right_plot(1,2))/(right_plot(2,1)-right_plot(1,1));

    %Make equations of left and right lines to extrapolate them
    % y= (m*x)+c
    xLeftY = 1; % x is on the left edge
    yLeftY = slopeL * (xLeftY - left_plot(1,1)) + left_plot(1,2);
    xRightY = 550; % x is on the right edge.
    yRightY = slopeL * (xRightY - left_plot(2,1)) + left_plot(2,2);
    
    xLeftW = 750; % x is on the left edge
    yLeftW = slopeR * (xLeftW - right_plot(1,1)) + right_plot(1,2);
    xRightW = 1300; % x is on the right edge.
    yRightW = slopeR * (xRightW - right_plot(2,1)) + right_plot(2,2);
    
    
    %folder = 'C:\Users\domin\OneDrive\Desktop\Lane Detection Algorithm\processed_images';
    %filename =  fullfile(folder, sprintf('processed%d.jpeg', i));
    imshow(img)
    hold on
    plot([xLeftY, xRightY], [yLeftY, yRightY], 'LineWidth',5,'Color','Yellow');
    plot([xLeftW, xRightW], [yLeftW, yRightW], 'LineWidth',5,'Color','White');
    %exportgraphics(gcf, filename);
    %writeVideo(Output_Video,getframe);


    clear lines;
    clear slopel;
    clear sloper;
    clear lines_Y;
    clear lines_W;
    clear rightp1;
    clear rightp2;
    clear leftp1;
    clear leftp2;
    clear output;
    pause(0.001)

end
%close(Output_Video)


