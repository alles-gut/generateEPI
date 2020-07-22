function getEPIdataset(filepath, epline)

addpath('lf-tools');
dpath     =  'lf-tools';
name_data =  'LF.mat';
trainpath = 'training';
labelpath = 'label';

load(fullfile(dpath,filepath,name_data));

[X,Y,Z] = getPointcloud(LF);
[R,G,B]  = getColor(fullfile(dpath,filepath));
R = reshape(R, 512, 512);
G = reshape(G, 512, 512);
B = reshape(B, 512, 512);

K = [[LF.f , 0    , size(X,1)/2 ];
     [ 0   , LF.f , size(X,2)/2 ];
     [ 0   , 0    , 1           ]];

%{ 
center = [LF.parameters.extrinsics.center_cam_x_m;
          LF.parameters.extrinsics.center_cam_y_m;
          LF.parameters.extrinsics.center_cam_z_m];
     
rotat = eul2rotm([LF.parameters.extrinsics.center_cam_rx_rad;
                  LF.parameters.extrinsics.center_cam_ry_rad;
                  LF.parameters.extrinsics.center_cam_rz_rad]);
%}
%{
eul = [LF.parameters.extrinsics.center_cam_rx_rad;
       LF.parameters.extrinsics.center_cam_ry_rad;
       LF.parameters.extrinsics.center_cam_rz_rad];
%}
camx = LF.parameters.extrinsics.num_cams_x;
camy = LF.parameters.extrinsics.num_cams_y;
baseline = LF.parameters.extrinsics.baseline_mm;
epi = [];
pose = [];
shift = -(camx-1)/2*baseline;
center = [0;0;0];
rotat = eul2rotm([0,0,0]);

m1 = [];
m2 = [];
for row = 1:1 %%
    Prow = mod(row+(camy-1)/2-1,camy);

    for i = 1:camx
        Pind = mod(i+(camx-1)/2-1,camx);
        P = center + [(Prow - (camy-1)/2) * baseline;Pind*baseline+shift;0];
        pose = [pose, [P; 1-1/camx*(i-1);0;0]];
    
        X_e = X(epline, :); Y_e = Y(epline, :); Z_e = Z(epline, :);
        R_e = R(epline, :); G_e = G(epline, :); B_e = B(epline, :);
        point3d = [X_e;Y_e;Z_e;ones(1, length(X))];
        
        proj = K * rotat * [eye(3), -P] * point3d;
        outproj = [[proj./proj(3,:)];R_e;G_e;B_e];
        outproj(1,:) = outproj(1,:) - min(outproj(1,:)) + 1;
        outproj(2,:) = outproj(2,:) - min(outproj(2,:)) + 1;
        m1 = [m1, max(outproj(1,:))];
        m2 = [m2, max(outproj(2,:))];

        if i == 1 && row == 1
            maxx = max(outproj(2,:));
            maxy = max(outproj(1,:));
        end
        if i <= (camx+1)/2
            if max(outproj(2,:)) > maxx
                outind = find(outproj(2,:) > maxx);
                outproj(:,outind) = [];
            end
        else
            if max(outproj(2,:)) > maxx
                adj_maxx = max(outproj(2,:)) - maxx;
                outind = find(outproj(2,:) - adj_maxx < 1);
                outproj(:,outind) = [];
                outproj(1,:) = outproj(1,:) - min(outproj(1,:)) + 1;
                outproj(2,:) = outproj(2,:) - min(outproj(2,:)) + 1;
            end
        end
        if row > (camy+1)/2
            if max(outproj(1,:)) > maxy
                outind = find(outproj(1,:) > maxy);
                outproj(:,outind) = [];
            end
        else
            if max(outproj(1,:)) > maxy
                adj_maxy = max(outproj(1,:)) - maxx;
                outind = find(outproj(1,:) - adj_maxy < 1);
                outproj(:,outind) = [];
                outproj(1,:) = outproj(1,:) - min(outproj(1,:)) + 1;
                outproj(2,:) = outproj(2,:) - min(outproj(2,:)) + 1;
            end
        end
    
        outproj(1,:) = (outproj(1,:) - 1)/(max(outproj(1,:)) - 1) * (size(X, 1)-1) + 1;
        outproj(2,:) = (outproj(2,:) - 1)/(max(outproj(2,:)) - 1) * (size(X, 2)-1) + 1;
    
        one = ones(1, length(outproj));
        epi = [epi, [outproj(2,:) ; one * (Pind+shift/baseline+(camx+1)/2) ; one ; outproj(4:6,:)]];
    end

    epi_img = zeros(camx, size(X,1), 3);
    for i = 1:length(epi)
        if round(epi(1,i)) <= size(X,1)
            epi_img(epi(2,i),round(epi(1,i)),:) = epi(4:6,i);
        end
    end

    filepath_ = strrep(filepath,'/','-');
    imwrite(epi_img, fullfile(trainpath,strcat(filepath_, int2str(Prow), '0', int2str(epline), '.png')));

    %Ground Truth
    epi_img_gt = getEPI(fullfile(dpath,filepath), camx, epline, ... 
        LF.parameters.intrinsics.image_resolution_x_px, Prow * camx + 1);
    imwrite(epi_img_gt, fullfile(labelpath,strcat(filepath_, int2str(Prow), '0', int2str(epline),'-label', '.png')));

end
end