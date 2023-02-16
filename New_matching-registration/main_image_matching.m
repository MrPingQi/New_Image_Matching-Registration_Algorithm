%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Gao Chenzhong
%   Beijing Key Laboratory of Fractional Signals and Systems,
%   Multi-Dimensional Signal and Information Processing Laboratory,
%   School of Information and Electronics, Beijing Institute of Technology
% Contact: gao-pingqi@qq.com
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [cor1,cor2] = main_image_matching(I1,I2,...
    int_flag,rot_flag,scl_flag,par_flag,key_type)

%% Parameters
G_resize = 2;  % Gaussian pyramid downsampling ratio, default: 2
nOctaves1 = 3; % Gaussian pyramid octave number, default: 3
nOctaves2 = 3; 
G_sigma = 1.6; % Gaussian blurring standard deviation, default: 1.6
nLayers = 4;   % Gaussian pyramid layer number, default: 4
radius = 1;    % Local non-maximum suppression radius, default: 2
N = 5000;      % Keypoints number threhold
patch_size = 72; % GGLOH patchsize, default: 72 or 96
NBS = 12;        % GGLOH localtion division, default: 12
NBO = 12;        % GGLOH orientation division, default: 12
Error = 5;       % Outlier removal pixel loss
K = 1;           % Rxperimental repetition times

%% Keypoints Detection
% LNMS Parameters of keypoints detection
% r1 = radius; r2 = radius; 
ratio = sqrt((size(I1,1)*size(I1,2))/(size(I2,1)*size(I2,2)));
if ratio>=1
    r2 = radius; r1 = round(radius*ratio);
else
    r1 = radius; r2 = round(radius/ratio);
end
tic,keypoints_1 = Detect_Keypoint(I1,6,r1,N,nOctaves1,G_resize,key_type);
    str=['Done: Keypoints detection of reference image, time cost: ',num2str(toc),'s\n']; fprintf(str);
%     figure,imshow(I1); hold on; plot(keypoints_1(:,1),keypoints_1(:,2),'r+'); pause(0.01)
if isempty(keypoints_1)
    cor1 = [];
    cor2 = [];
    return
end
tic,keypoints_2 = Detect_Keypoint(I2,6,r2,N,nOctaves2,G_resize,key_type);
    str=['Done: Keypoints detection of sensed image, time cost: ',num2str(toc),'s\n']; fprintf(str);
%     figure,imshow(I2); hold on; plot(keypoints_2(:,1),keypoints_2(:,2),'r+'); pause(0.01)
if isempty(keypoints_2)
    cor1 = [];
    cor2 = [];
    return
end

%% Keypoints Description
tic,descriptors_1 = Multiscale_Descriptor(I1,keypoints_1,patch_size,NBS,NBO,...
    nOctaves1,nLayers,G_resize,G_sigma,int_flag,rot_flag,par_flag);
    str=['Done: Keypoints description of reference image, time cost: ',num2str(toc),'s\n']; fprintf(str);
tic,descriptors_2 = Multiscale_Descriptor(I2,keypoints_2,patch_size,NBS,NBO,...
    nOctaves2,nLayers,G_resize,G_sigma,int_flag,rot_flag,par_flag);
    str=['Done: Keypoints description of sensed image, time cost: ',num2str(toc),'s\n']; fprintf(str);

%% Keypoints Matching
tic
if K==1
    [cor1_o,cor2_o] = Multiscale_Matching(descriptors_1,descriptors_2,...
        nOctaves1,nOctaves2,nLayers,scl_flag,par_flag);
    [cor1,cor2] = Outlier_Removal(cor1_o,cor2_o,Error);
else
    aaa = []; correspond_1 = cell(K,1); correspond_2 = cell(K,1);
    for k = 1:K
        [cor1_o,cor2_o] = Multiscale_Matching(descriptors_1,descriptors_2,...
            nOctaves1,nOctaves2,nLayers,scl_flag,par_flag);
        [cor1,cor2] = Outlier_Removal(cor1_o,cor2_o,Error);
        correspond_1{k} = cor1; correspond_2{k} = cor2; aaa = [aaa,size(cor1,1)];
    end
    [~,index] = max(aaa);
    cor1 = correspond_1{index}; cor2 = correspond_2{index};
end
    str = ['Done: Keypoints matching, time cost: ',num2str(toc),'s\n\n']; fprintf(str);