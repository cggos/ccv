clearvars;
%% Path setting
ROOT = 'E:\new_pet\analyze3\data/';
rawSrcRootPath_1 = [ROOT, 'FDG/'];
% rawSrcRootPath_2 = [ROOT, 'AV45_FA_realign/'];
% rawSrcRootPath_3 = [ROOT, 'TAU_FT_realign/'];

% rawSrcRootPath_1 = [ROOT, 'FDG_1/'];
% rawSrcRootPath_2 = [ROOT, 'AV45_1/'];
% rawSrcRootPath_3 = [ROOT, 'TAU_1/'];

%% Normalize 
niftiSubs_1 = dir(rawSrcRootPath_1);
% niftiSubs_2 = dir(rawSrcRootPath_2);
% niftiSubs_3 = dir(rawSrcRootPath_3);

niftiSubs_1 = niftiSubs_1(3:end);
% niftiSubs_2 = niftiSubs_2(3:end);
% niftiSubs_3 = niftiSubs_3(3:end);

for i = 1:numel(niftiSubs_1)
    
%     niftiSub_1 = dir(fullfile(rawSrcRootPath_1, niftiSubs_1(i).name, '*.nii'));
%     niftiSub_2 = dir(fullfile(rawSrcRootPath_2, niftiSubs_2(i).name, '*.nii'));
%     niftiSub_3 = dir(fullfile(rawSrcRootPath_3, niftiSubs_3(i).name, '*.nii'));

    disp(fullfile(rawSrcRootPath_1, niftiSubs_1(i).name));%输出看一下
%     disp(fullfile(rawSrcRootPath_2, niftiSubs_2(i).name));
%     disp(fullfile(rawSrcRootPath_3, niftiSubs_3(i).name));

    data_1 = char(strcat(fullfile(rawSrcRootPath_1, niftiSubs_1(i).name),',1'));
%     data_2 = char(strcat(fullfile(rawSrcRootPath_2, niftiSubs_2(i).name),',1'));
%     data_3 = char(strcat(fullfile(rawSrcRootPath_3, niftiSubs_3(i).name),',1'));

    spm_jobman('initcfg');
    matlabbatch = {};

    matlabbatch{1}.spm.tools.oldnorm.estwrite.subj.source = {data_1};
    matlabbatch{1}.spm.tools.oldnorm.estwrite.subj.wtsrc = '';
    matlabbatch{1}.spm.tools.oldnorm.estwrite.subj.resample = {data_1;data_2};
    matlabbatch{1}.spm.tools.oldnorm.estwrite.eoptions.template = {'E:\spm12\spm12\toolbox\OldNorm\PET.nii,1'};
    matlabbatch{1}.spm.tools.oldnorm.estwrite.eoptions.weight = '';
    matlabbatch{1}.spm.tools.oldnorm.estwrite.eoptions.smosrc = 8;
    matlabbatch{1}.spm.tools.oldnorm.estwrite.eoptions.smoref = 0;
    matlabbatch{1}.spm.tools.oldnorm.estwrite.eoptions.regtype = 'mni';
    matlabbatch{1}.spm.tools.oldnorm.estwrite.eoptions.cutoff = 25;
    matlabbatch{1}.spm.tools.oldnorm.estwrite.eoptions.nits = 16;
    matlabbatch{1}.spm.tools.oldnorm.estwrite.eoptions.reg = 1;
    matlabbatch{1}.spm.tools.oldnorm.estwrite.roptions.preserve = 0;
    matlabbatch{1}.spm.tools.oldnorm.estwrite.roptions.bb = [-78 -112 -70
                                                            78 76 85];
    matlabbatch{1}.spm.tools.oldnorm.estwrite.roptions.vox = [2 2 2];
    matlabbatch{1}.spm.tools.oldnorm.estwrite.roptions.interp = 1;
    matlabbatch{1}.spm.tools.oldnorm.estwrite.roptions.wrap = [0 0 0];
    matlabbatch{1}.spm.tools.oldnorm.estwrite.roptions.prefix = '@';

    spm_jobman('run',matlabbatch);
    clear matlabbatch;
    
    %% 将标准化之后新生成的a打头的图像移动到新的文件夹
    realignSub_1 = fullfile(ROOT, 'FDG_norm');
%     realignSub_2 = fullfile(ROOT, 'AV45_FA_norm');
%     realignSub_3 = fullfile(ROOT, 'TAU_FT_norm');
    
    %创建指定文件夹
    mkdir(realignSub_1);
    movefile(fullfile(rawSrcRootPath_1,'@*'), realignSub_1);

%     mkdir(realignSub_2);
%     movefile(fullfile(rawSrcRootPath_2,'@*'), realignSub_2);
% 
%     mkdir(realignSub_3);
%     movefile(fullfile(rawSrcRootPath_3,'@*'), realignSub_3);

end
