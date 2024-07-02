%%********************************************************************
% FOCAL Gas Deck Shield (gds) USB Test           May 9, 2023 1:12 PM
% Using Adafruit Feather M4 CAN Express
%
% USB-based test of Gas Deck Shield functions. 
% 

cd C:\huarp\Carbon\FOCAL2023\code\gds\Carbon_GDS\Matlab
% cd C:\Users\nort\Documents\Documents\Exp\Boards\  ???  \Matlab
%%
serial_port_clear();
%%
[s] = serial_port_init(); 
%   Should not need to set Baud Rate 
% set(s,'BaudRate',57600);
% set(s,'BaudRate',115200);
%%
% First check that the board is a Gas Deck Shield 
[subfunc,desc] = get_subfunction(s);
if subfunc ~= 18
  error('Expected subfunction 18 for Feather M4 CAN Express Gas Deck Shield. Reported %d', subfunc);
end
BoardID = read_subbus(s,2);
Build = read_subbus(s,3);
[SerialNo,SNack] = read_subbus(s,4);
[InstID,InstIDack] = read_subbus(s,5);

% Rev set to A for GDS until Rev B
Rev = 'A';

if BoardID == 1
  BdCfg = 'Gas Deck Shield';
elseif BoardID == 2
  BdCfg = 'CO2 OE ??? Shield';
elseif ( BoardID == 3 )
  BdCfg = 'MM OE ??? Shield';
else
  BdCfg = 'Test';
end

fprintf(1, 'Attached to GDS S/N %d Build # %d\n', SerialNo, Build);
fprintf(1, 'Board is Rev %s configured as "%s"\n', Rev, BdCfg);
fprintf(1, 'The description is "%s"\n', desc);

rm_obj = read_multi_prep([8,40,9,0]);
[vals,~] = read_multi(s,rm_obj);
even = mod(vals(2:end),256);
odd = floor(vals(2:end)/256);
il = [even odd]';
il = il(:)';
nc = find(il == 0,1);
il = il(1:(nc-1));
desc = char(il);
fprintf(1,'Description from FIFO is: %s\n', desc);
%fprintf(1, 'Now figure out how to interpret the result\n');

%%
% Read the ADS1115 ADC Section
adc_adr = 32;   % 0x20 
adc_nch = 16;   % 16 channels total
rm_obj = read_multi_prep([adc_adr,1,(adc_adr+adc_nch+1)]); % [0x20,1,0x31]
%
% while true
for iadc=1:10
  
  [vals,~] = read_multi(s,rm_obj);
  
  fprintf(1,'--------- Read %d ---------\n', iadc);
  fprintf(1,'Status %04X : N_Reads %d\n', vals(1),vals(end));
  adc = vals(2:end-1);
  sadc = adc - (adc>2^15)*2^16;
% Vref is individually set for each channel: 
  vref = [ 6.144; 6.144; 6.144; 6.144; 6.144; 6.144; 4.096; 4.096; 4.096; 4.096; 4.096; 4.096; 4.096; 4.096; 4.096; 4.096 ];
  vadc = vref .* sadc / (2^15);
  Rpu = 75e3;
  Rth = Rpu * vadc ./ (vref-vadc);
  for i=1:length(adc)
    fprintf(1,'%8X %8d %10f V %10g Ohm\n', adc(i), adc(i), vadc(i), Rth(i));
  end
  %
  pause(1);
end

%%
% Read from Timer section
rm_obj = read_multi_prep([64,1,67]); % 0x40
%
TC_F = 1e5;
% TC_F = 8e6; % for testing: ~9 Minutes to rollover
T0 = -1;
iT0 = -1;
N = 100;
Tsec = 20;
Psec = Tsec/N;
for ielp = 0:N
  [vals,~] = read_multi(s,rm_obj);
  T1 = vals(1) + vals(2)*65536;
  if T0 >= 0
    dT = (T1-T0)/TC_F;
    TT = (T1-iT0)/TC_F;
    fprintf(1, 'Elapsed/Loop/Max/State: TT:%.3f s dT:%.5f s LT:%.3f ms LMT:%.3f ms\n', TT, dT, vals(3)/TC_F*1e3, vals(4)/TC_F*1e3);
  else
    iT0 = T1;
    tic;
    fprintf(1, 'Starting value %.6f sec\n', iT0/TC_F);
  end
  T0 = T1;
  if ielp < N; pause(Psec); end
end
telapsed = toc;
iT1 = T1;
ips = ((iT1-iT0)/TC_F)/telapsed;
fprintf(1, 'Observed %.5f seconds/seconds\n', ips);
%%
N = 1000;
curlooptime = zeros(N,1);
for i=1:N
  val = read_subbus(s,66);
  curlooptime(i) = (val/TC_F)*1e3; % msec
  pause(.1);
end
%
figure; plot(curlooptime,'.');
ylabel('msec');

%%
% On-Board MS8607 PTRH :
ms8_base = hex2dec('60'); % 0x60

% Read Coefficients
rm_obj = read_multi_prep([ms8_base+5,1,ms8_base+10]);  % [0x65 - 0x6A]
[vals,~] = read_multi(s,rm_obj);
%
if isempty(vals) || length(vals) ~= 6 
  error('vals length was %d, expected 6', length(vals));
end
%
fprintf(1, '\nMS8607 Coefficients:\n');
for i=1:6
  fprintf(1, '  C%d: %x  %d\n', i, vals(i), vals(i));
end
%% P, T, and RH
%
rm_obj = read_multi_prep([ms8_base+1,1,ms8_base+16]); % 0x61 - 0x70

fprintf(1, '\nMS8607 Pressure, Temperature, and Relative Humidity:\n');
for i=0:9
  [vals,ack] = read_multi(s, rm_obj);
  
PTRHread = struct( ...
  'T', { typecast(uint32(vals(3)+65536*vals(4)),'single') }, ...
  'P', { typecast(uint32(vals(1)+65536*vals(2)),'single') }, ...
  'RH', { vals(16) });
  
  fprintf(1,'P%d: %7.3f mBar ( %7.3f Torr )  T%d: %7.3f degC  RH%d: %5.2f %%\n', ...
    i, PTRHread.P, (PTRHread.P * 0.750062), i, PTRHread.T, i, PTRHread.RH/100);
 
  pause(1);
end

%%
% Command Testing 
cmd_adr = 16;  % 0x10 
cmd_pins = { 'CAL_HI', 'CAL_LO', 'CAL_REF', 'CO2_REF', 'CAL_SPR', 'MM_PUMP', 'MM_EXH', 'CO2_PUMP', 'CO2_EXH', 'CKT3_EN', 'INV_ARM'
 };
for npin = 1:length(cmd_pins)
  pin = cmd_pins{npin};
  cmdnum = (npin-1)*2+1;
  fprintf(1, 'Hit ENTER to command %s ON (cmd %d)\n', pin, cmdnum);
  pause;
  write_subbus(s, cmd_adr, cmdnum);
  status = read_subbus(s, cmd_adr);
  fprintf(1, '  Status is %04X\n', status);
  cmdnum = (npin-1)*2;
  fprintf(1, 'Hit ENTER to command %s OFF (cmd %d)\n', pin, cmdnum);
  pause;
  write_subbus(s, cmd_adr, cmdnum);
  status = read_subbus(s, cmd_adr);
  fprintf(1, '  Status is %04X\n', status);
end

%%
% Subbus fail test
tick_subbus(s);
write_subbus(s, 6, 0); % Clear fail
failval = read_subbus(s,6);
if failval
  fprintf(1,'Fail is non-zero after tick and explicit clear\n');
end
%%
fprintf(1, 'Testing default timeout of 120 seconds:\n');
write_subbus(s, 6, 0); % Clear fail
tick_subbus(s, 1);
tic;
failval = read_subbus(s,6);
faildur = toc;
while failval == 0 && faildur < 125
  pause(1);
  failval = read_subbus(s,6);
  faildur = toc;
end
if failval
  if faildur < 119
    fprintf(1,'ERROR: Fail occurred early: %f secs\n', faildur);
  else
    fprintf(1, 'Fail occurred after %f secs, within specs\n', faildur);
  end
else
  fprintf(1, 'ERROR: Timed out after %f secs without fail\n', faildur);
end
%%
fprintf(1, 'Testing reduced timeout of 20 seconds:\n');
tick_subbus(s,1);
write_subbus(s, 6, 100*256+0); % Clear fail
tic;
failval = bitand(read_subbus(s,6),255);
faildur = toc;
while failval == 0 && faildur < 25
  pause(1);
  failval = bitand(read_subbus(s,6),255);
  faildur = toc;
end
if failval
  if faildur < 19
    fprintf(1,'ERROR: Fail occurred early: %f secs\n', faildur);
  else
    fprintf(1, 'Fail occurred after %f secs, within specs\n', faildur);
  end
else
  fprintf(1, 'ERROR: Timed out after %f secs without fail\n', faildur);
end
