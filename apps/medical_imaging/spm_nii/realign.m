clearvars;
%% Path setting
ROOT = 'E:\new_pet\analyze3\data/';
rawSrcRootPath_1 = [ROOT, 'FDG_FA/'];
rawSrcRootPath_2 = [ROOT, 'AV45_FA/'];
% rawSrcRootPath_3 = [ROOT, 'TAU_FT/'];

%% realign 把AV45\TAU的图像都按照FDG的图像进行头动矫正   
niftiSubs_1 = dir(rawSrcRootPath_1);
niftiSubs_2 = dir(rawSrcRootPath_2);
% niftiSubs_3 = dir(rawSrcRootPath_3);

niftiSubs_1 = niftiSubs_1(3:end);
niftiSubs_2 = niftiSubs_2(3:end);
% niftiSubs_3 = niftiSubs_3(3:end);

for i = 1:numel(niftiSubs_1)
    
    niftiSub_1 = dir(fullfile(rawSrcRootPath_1, niftiSubs_1(i).name, '*.nii'));
    niftiSub_2 = dir(fullfile(rawSrcRootPath_2, niftiSubs_2(i).name, '*.nii'));
%     niftiSub_3 = dir(fullfile(rawSrcRootPath_3, niftiSubs_3(i).name, '*.nii'));

    spm_jobman('initcfg');
    matlabbatch = {};

    disp(fullfile(rawSrcRootPath_1, niftiSubs_1(i).name));
    disp(fullfile(rawSrcRootPath_2, niftiSubs_2(i).name));
%     disp(fullfile(rawSrcRootPath_3, niftiSubs_3(i).name));
    
%     data_1 = strcat(fullfile(rawSrcRootPath_1, niftiSubs_1(i).name), ',1');
%     data_2 = strcat(fullfile(rawSrcRootPath_2, niftiSubs_2(i).name), ',1');
%     data_3 = strcat(fullfile(rawSrcRootPath_3, niftiSubs_3(i).name), ',1');

    data_1 = char(strcat(fullfile(rawSrcRootPath_1, niftiSubs_1(i).name),',1'));
    data_2 = char(strcat(fullfile(rawSrcRootPath_2, niftiSubs_2(i).name),',1'));
%     data_3 = char(strcat(fullfile(rawSrcRootPath_3, niftiSubs_3(i).name),',1'));


    matlabbatch{1}.spm.spatial.realign.estwrite.data = {

                                                        {
                                                        data_1;data_2
%                                                         'new_pet\FDG_1\CHEN_LAI_FA_20180622_12197_006_1_4_PTCT_Brain_Brain_MAC.nii,1',
%                                                         'new_pet\AV45_1\CHEN_LAI_FA_20180627_12266_006_1_12_PTCT_Brain_AV45_Brain_MAC.nii,1',
%                                                         'new_pet\TAU_1\CHEN_LAI_FA_20180621_12186_006_1_11_PTCT_Brain_AV1451_Brain_MAC.nii,1'
                                       
                                                        }
                                                        };
    matlabbatch{1}.spm.spatial.realign.estwrite.eoptions.quality = 0.9;
    matlabbatch{1}.spm.spatial.realign.estwrite.eoptions.sep = 4;
    matlabbatch{1}.spm.spatial.realign.estwrite.eoptions.fwhm = 5;
    matlabbatch{1}.spm.spatial.realign.estwrite.eoptions.rtm = 1;
    matlabbatch{1}.spm.spatial.realign.estwrite.eoptions.interp = 2;
    matlabbatch{1}.spm.spatial.realign.estwrite.eoptions.wrap = [0 0 0];
    matlabbatch{1}.spm.spatial.realign.estwrite.eoptions.weight = '';
    matlabbatch{1}.spm.spatial.realign.estwrite.roptions.which = [2 1];
    matlabbatch{1}.spm.spatial.realign.estwrite.roptions.interp = 4;
    matlabbatch{1}.spm.spatial.realign.estwrite.roptions.wrap = [0 0 0];
    matlabbatch{1}.spm.spatial.realign.estwrite.roptions.mask = 1;
    matlabbatch{1}.spm.spatial.realign.estwrite.roptions.prefix = 'a';
    
    %执行matlabbatch
    spm_jobman('run',matlabbatch);
    clear matlabbatch;
    
    %% 将头动配准后新生成的a打头的图像移动到新的文件夹
    realignSub_1 = fullfile(ROOT, 'FDG_FA_realign');
    %输出是ROOT/FDG_all_realign
    realignSub_2 = fullfile(ROOT, 'AV45_FA_realign');
%     realignSub_3 = fullfile(ROOT, 'TAU_FT_realign');
    
    %创建指定文件夹
    mkdir(realignSub_1);
    movefile(fullfile(rawSrcRootPath_1,'a*'), realignSub_1);

    mkdir(realignSub_2);
    movefile(fullfile(rawSrcRootPath_2,'a*'), realignSub_2);

%     mkdir(realignSub_3);
%     movefile(fullfile(rawSrcRootPath_3,'a*'), realignSub_3);

end