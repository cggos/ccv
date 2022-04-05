clearvars;

templatePath = 'E:\new_pet\preprocess\temp_new\Resliced_tempwhole.nii';
templateVol = spm_read_vols(spm_vol(templatePath));

outFile = fopen('E:\new_pet\analyze3\data/TAU_suvrForVoxelInNii2.csv', 'a');
root = 'E:\new_pet\analyze3\data\TAU_suvrforvoxel';
     
subs = dir(root);
subs = subs(3:end);
for i = 1:numel(subs)
    tmpNiis = dir(fullfile(root, subs(i).name, '*.nii'));
    for j = 1:numel(tmpNiis)
        fprintf(outFile, '%s', subs(i).name);
        tmpNifti = spm_vol(fullfile(root, subs(i).name, tmpNiis(j).name));
        tmpVol = spm_read_vols(tmpNifti);
%         a = (tmpVol==4);
        for k = 1:108
            tmpBinTemplate = templateVol;
            tmpBinTemplate(tmpBinTemplate~=k) = 0;
            tmpBinTemplate(tmpBinTemplate==k) = 1;
            tmpBinTemplate(tmpBinTemplate ~= 0);
            tmpCurVol = tmpVol;
            tmpCurVol = tmpBinTemplate .* tmpCurVol;
            fprintf(outFile, ',%f', mean(tmpCurVol(tmpCurVol~=0)));
        end
        fprintf(outFile, '\n');
    end
end
fclose(outFile);

clearvars;