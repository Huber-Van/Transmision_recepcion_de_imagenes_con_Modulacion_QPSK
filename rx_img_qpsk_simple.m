function img = rx_img_qpsk_simple(inSc16)
% RX QPSK robusto (sin CMA):
% RRC -> SymbolSync(ZC) -> Barker -> ajuste CFO/φ con preámbulo
% -> entrenamiento QPSK (64 sym) para refinar CFO/φ
% -> búsqueda header {conj?, rot 0/90/180/270, slip 0..3}
% -> PLL (QPSK) en payload + snap π/4
% -> QPSK(π/4,Gray) -> DESCRAMBLER -> imagen (MSB)
% Requiere: load_sc16q11()

%% ===== Parámetros =====
sps=8; rolloff=0.35; span=10; phase0=pi/4;
lenBark=13; repBark=2; Lpre=lenBark*repBark;
Ntrain=64;                               % debe coincidir con TX
HDR_BITS=56;                             % 16+16+8+16
fprintf('\n=== RX QPSK estable ===\n');

%% ===== 1) Cargar y normalizar =====
rx=double(load_sc16q11(inSc16)); rx=rx(:);
rx=rx-mean(rx); rx=rx/max(1e-12,rms(rx));

%% ===== 2) RRC =====
rrcRx = comm.RaisedCosineReceiveFilter('Shape','Square root', ...
    'RolloffFactor',rolloff,'FilterSpanInSymbols',span, ...
    'InputSamplesPerSymbol',sps,'DecimationFactor',1);
rx_f = rrcRx([rx; zeros(span*sps,1)]); rx_f=rx_f(span*sps+1:end);

%% ===== 3) SymbolSync (ZC) =====
symSync = comm.SymbolSynchronizer('TimingErrorDetector','Zero-Crossing (decision-directed)', ...
  'SamplesPerSymbol',sps,'DampingFactor',1.0,'NormalizedLoopBandwidth',0.006);
rx_sym = symSync(rx_f);  % 1 sps

%% ===== 4) Barker =====
barker=comm.BarkerCode('Length',lenBark,'SamplesPerFrame',lenBark);
barkBits=(1+barker())/2; preBits=repmat(barkBits,repBark,1);
preSymB=pskmod(preBits,2,0);
c=filter(flipud(conj(preSymB)),1,rx_sym); [pk,ix]=max(abs(c));
startPre = ix-Lpre+1;  if startPre<1, error('No Barker'); end
fprintf('Barker @ %d (%.2f)\n', startPre, pk);

%% ===== 5) CFO/φ con preámbulo =====
z=rx_sym(startPre:startPre+Lpre-1).*conj(preSymB);
phi=unwrap(angle(z(:))).'; k=0:Lpre-1; p=polyfit(k,phi,1);
alpha=p(1); beta=p(2);
m=(0:length(rx_sym)-startPre).';
rx_fix = rx_sym(startPre:end).*exp(-1j*(alpha*m+beta));
r_pre   = rx_fix(1:Lpre);                 % BPSK preámbulo
r_after0= rx_fix(Lpre+1:end);             % resto

%% ===== 6) ENTRENAMIENTO QPSK para refinar (64 símbolos) =====
pat16 = pskmod([0;1;2;3; 1;0;3;2; 2;3;0;1; 3;2;1;0], 4, phase0, 'gray');
trainSym = repmat(pat16,4,1);
r_train = r_after0(1:Ntrain);
best = struct('var',inf,'conj',false,'a',0,'b',0);
for cc=[false true]
    t = r_train; if cc, t=conj(t); end
    ph = unwrap(angle(t .* conj(trainSym)));
    k  = 0:Ntrain-1; p = polyfit(k,ph.',1);
    v = var(ph.' - polyval(p,k));
    if v<best.var, best=struct('var',v,'conj',cc,'a',p(1),'b',p(2)); end
end
r_after = r_after0(Ntrain+1:end);
if best.conj, r_after = conj(r_after); end
n=(0:numel(r_after)-1).';
r_after = r_after .* exp(-1j*(best.a*n + best.b));

%% ===== 7) Buscar HEADER {rot, slip} (conj ya elegido) =====
segN=min(numel(r_after),200000);
phiC=[0,pi/2,pi,3*pi/2]; slipC=0:3;
ok=false; bestH=0;bestW=0;bestC=0;rotSel=0;slipSel=0;
for ss=slipC
    if 1+ss>segN, continue; end
    s=r_after(1+ss:segN);
    for rr=1:numel(phiC)
        sy = s.*exp(-1j*phiC(rr));
        b  = pskdemod(sy,4,phase0,'gray','OutputType','bit'); bits=b(:).';
        if numel(bits)<HDR_BITS, continue; end
        hdr   = bits(1:HDR_BITS);
        H     = bi2de(hdr(1:16) ,'left-msb');
        W     = bi2de(hdr(17:32),'left-msb');
        C     = bi2de(hdr(33:40),'left-msb');
        okcrc = isequal(crc16(hdr(1:40)), hdr(41:56));
        if okcrc && H>=1 && H<=8192 && W>=1 && W<=8192 && ismember(C,[1,3])
            ok=true; bestH=H;bestW=W;bestC=C; rotSel=phiC(rr); slipSel=ss; break;
        end
    end
    if ok, break; end
end
if ~ok, error('Header inválido'); end
fprintf('Header OK: %dx%dx%d | conj=%d rot=%.2f slip=%d\n',bestH,bestW,bestC,best.conj,rotSel,slipSel);

%% ===== 8) Extraer secuencias para figuras y payload =====
% Header symbols (para gráfica exclusiva QPSK-HEADER):
hdr_syms = r_after(1+slipSel : slipSel + ceil(HDR_BITS/2));
hdr_syms = hdr_syms .* exp(-1j*rotSel);

% Payload (después del header):
s_all = r_after(1+slipSel+ceil(HDR_BITS/2) : end) .* exp(-1j*rotSel);

% Copia para figura "antes de tracking":
s_before = s_all;

%% ===== 9) PLL QPSK en PAYLOAD + "snap" π/4 =====
carSync2 = comm.CarrierSynchronizer('Modulation','QPSK', ...
  'ModulationPhaseOffset','Custom','CustomPhaseOffset',phase0, ...
  'SamplesPerSymbol',1,'DampingFactor',0.707,'NormalizedLoopBandwidth',0.001);
s_all = carSync2(s_all);

K=min(3000,numel(s_all));
phi_meas = angle(mean(s_all(1:K).^4))/4;
kpi2 = round((phi_meas - phase0)/(pi/2));
phi_fix = phi_meas - (phase0 + kpi2*(pi/2));
s_all = s_all .* exp(-1j*phi_fix);

%% ===== 10) Demod -> quitar header -> DESCRAMBLER -> bytes =====
b_all = pskdemod([hdr_syms; s_all],4,phase0,'gray','OutputType','bit'); b_all=b_all(:).';
imgBits = b_all(HDR_BITS+1:end);
imgBits = lfsr_descramble(imgBits, 127);

needBytes = double(bestH)*double(bestW)*double(max(1,bestC));
needBits  = needBytes*8;
if numel(imgBits)<needBits, imgBits(end+1:needBits)=0; else, imgBits=imgBits(1:needBits); end
bytes = uint8(bi2de(reshape(imgBits,8,[]).','left-msb'));

if bestC==1, img=reshape(bytes,[bestH bestW]);
else,         img=reshape(bytes,[bestH bestW bestC]); end

%% ===== Mostrar =====
figure('Name','Imagen RX'); imshow(img);
title(sprintf('Imagen RX QPSK (%dx%dx%d)',bestH,bestW,bestC));

% --- Figuras separadas BPSK / QPSK ---
try
    % BPSK: constelación preámbulo
    figure('Name','BPSK: Preámbulo (constelación)');
    plot(real(r_pre), imag(r_pre), '.'); axis equal; grid on;
    title('BPSK preámbulo (debe caer en el eje real)');

    % BPSK: bits (demodulacion del preámbulo)
    b_pre = pskdemod(r_pre,2,0,'OutputType','bit');
    figure('Name','BPSK: Bits del preámbulo');
    stem(b_pre(1:min(end,2*lenBark*repBark)),'filled'); xlabel('Símbolo'); ylabel('bit'); grid on;
    title('Demod BPSK del preámbulo Barker');

    % QPSK: header (solo cabecera)
    figure('Name','QPSK: Header (constelación)');
    plot(real(hdr_syms(1:min(end,2000))), imag(hdr_syms(1:min(end,2000))), '.');
    axis equal; grid on; title('QPSK (π/4, Gray) — Header');

    % QPSK: payload antes y después del tracking
    figure('Name','QPSK: Payload (antes de tracking)');
    plot(real(s_before(1:min(end,8000))), imag(s_before(1:min(end,8000))), '.');
    axis equal; grid on; title('Payload antes del PLL');

    figure('Name','QPSK: Payload (después de tracking)');
    plot(real(s_all(1:min(end,8000))), imag(s_all(1:min(end,8000))), '.');
    axis equal; grid on; title('Payload después del PLL + snap π/4');
catch, end
end

%% ===== Descrambler / CRC =====
function bitsOut = lfsr_descramble(bitsIn, seed)
s = de2bi(seed,7,'left-msb'); s=s(:).';
bitsOut = false(size(bitsIn));
for n=1:numel(bitsIn)
    fb = xor(s(4), s(7));
    bitsOut(n) = xor(bitsIn(n), fb);
    s = [fb s(1:6)];
end
end

function crc = crc16(bits)
crcReg=uint16(hex2dec('FFFF'));
for i=1:numel(bits)
  inbit=logical(bits(i)); xorbit=bitget(crcReg,1)~=inbit;
  crcReg=bitshift(crcReg,-1);
  if xorbit, crcReg=bitxor(crcReg,hex2dec('A001')); end
end
crc=de2bi(crcReg,16,'left-msb');
end
