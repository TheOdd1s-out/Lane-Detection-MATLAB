clc;
clear;
close all;
% Read Input Image

% Code for region of Interest
InputImage=imread('out10.jpg');

% Display the Image

imshow(InputImage);

% Get Inputs from Mouse,Select 18 Seed Points in Image
[x, y]=ginput(18);
% Select polygonal region of interest
BinaryMask = roipoly(InputImage,x,y);
imshow(BinaryMask)