function tick_subbus(s, quiet)
fprintf(s, 'T');
if nargin > 1 && quiet
  return
end
tline = fgetl(s);
if isempty(tline)
  fprintf(1,'tick_subbus timeout with no reply (as expected)\n');
elseif tline(1) ~= '0'
  fprintf(1,'ERROR: tick_subbus received return string "%s"\n', tline);
end
