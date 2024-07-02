function str = words2txt(data)
even = mod(data,256);
odd = floor(data/256);
il = [even odd]';
il = il(:)';
nc = find(il == 0,1);
il = il(1:(nc-1));
str = char(il);
