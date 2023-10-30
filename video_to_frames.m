% Specify the video file
videoFile = 'C:\Users\jason\Downloads\Lane-Detection-MATLAB-main (3)\Lane-Detection-MATLAB-main\input_video.mp4';

% Create a VideoReader object
video = VideoReader(videoFile);

% Directory to store the frames
outputDir = 'C:\Users\jason\Downloads\Lane-Detection-MATLAB-main (3)\Lane-Detection-MATLAB-main\Frames';
if ~exist(outputDir, 'dir')
   mkdir(outputDir);
end

% Read and write each frame
frameCount = 0;
while hasFrame(video)
    frameCount = frameCount + 1;
    img = readFrame(video);
    frameFilename = fullfile(outputDir, sprintf('out%d.jpg', frameCount));
    imwrite(img, frameFilename);
    fprintf('Wrote frame %d to %s\n', frameCount, frameFilename);
end

fprintf('Finished writing %d frames from %s to %s\n', frameCount, videoFile, outputDir);
