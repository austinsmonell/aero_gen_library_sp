function [lambda1, lambda2] = get_lambda(y, ref1 , ref2)

pts1 = [y(1), y(2), y(3)]';
pts2 = [y(13), y(14), y(15)]';
center_1 = ref1(:, 1);
ref_vec_1 = ref1(:, 2);
center_2 = ref2(:, 1);
ref_vec_2 = ref2(:, 2);

%% Center
pos_center_1 = normalize(pts1 - center_1, 'norm');
% ref_vec = ones(size(pos_center)).*normalize(R_el*R_az*[r, 0, -b]' - center, 'norm');

sign_val_1 = cross(ref_vec_1, pos_center_1);
sign_val_1 = sign_val_1(1, :)./abs(sign_val_1(1, :));

lambda1 = dot(pos_center_1, ref_vec_1)./(sqrt(dot(pos_center_1, pos_center_1)).*sqrt(dot(ref_vec_1, ref_vec_1)));
lambda1 = sign_val_1.*real(acos(lambda1));


pos_center_2 = normalize(pts2 - center_2, 'norm');
% ref_vec = ones(size(pos_center)).*normalize(R_el*R_az*[r, 0, -b]' - center, 'norm');

sign_val_2 = cross(ref_vec_2, pos_center_2);
sign_val_2 = sign_val_2(1, :)./abs(sign_val_2(1, :));

lambda2 = dot(pos_center_2, ref_vec_2)./(sqrt(dot(pos_center_2, pos_center_2)).*sqrt(dot(ref_vec_2, ref_vec_2)));
lambda2 = sign_val_2.*real(acos(lambda2));

end