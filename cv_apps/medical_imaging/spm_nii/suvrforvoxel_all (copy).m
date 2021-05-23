clearvars;
%����FDG PET����ݱ�׼����ĵõ�AV45 TAU����С�����
% ��ֵ��ģ��
maskPath = '../AAL_cerebellar.nii';
tmp = spm_vol(maskPath);
maskVol = spm_read_vols(tmp);
maskVol(maskVol~=0) = 1;    

% ��normalize֮���Ӱ��Ͷ�ֵ����ģ���ˣ��õ����Ը�voxel����ȡֵ��
% Ȼ�����С�Ե�ƽ��ֵ���õ�����voxel��ƽ����ȡ�ʣ���suvr
niiRootPath = 'E:\new_pet\analyze3\data/AV45_smooth';
outRootPath = 'E:\new_pet\analyze3\data/AV45_smooth_suvrForVoxelInNii';
subs = dir(niiRootPath);%�г�ָ��Ŀ¼���������ļ��к��ļ�
subs = subs(3:end);

% tmpSubNiis = dir(fullfile('..', '*.nii'));

for i = 1:numel(subs)
    tmpSubNiis = dir(fullfile(niiRootPath, subs(i).name, '*.nii'));
    for j = 1:numel(tmpSubNiis)
        tmpNii = spm_vol(fullfile(niiRootPath, subs(i).name, tmpSubNiis(j).name));
        tmpVol = spm_read_vols(tmpNii);
        tmpResVol = maskVol .* tmpVol;
        tmpResVol = tmpVol / mean(tmpResVol(tmpResVol~=0));
        mkdir(fullfile(outRootPath, subs(i).name));
        tmpNii.fname = fullfile(outRootPath, subs(i).name, [subs(i).name,'suvr_', tmpSubNiis(j).name]);%�������ļ���,����Ḳ��ԭ�ļ�
        tmpNii.dt = [64, 0];
        spm_write_vol(tmpNii, tmpResVol);
    end
end
clearvars;