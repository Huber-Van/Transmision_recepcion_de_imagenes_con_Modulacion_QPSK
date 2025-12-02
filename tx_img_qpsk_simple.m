function tx_img_qpsk_simple(imgPath, outSc16)
% TX QPSK(π/4, Gray) robusto y "format-friendly" para JPG/JPEG/PNG.
% Señal a 1 sps antes del RRC:
%   [ BPSK Barker-13×2 ] + [ QPSK training 64 sym ] + [ QPSK HEADER ] + [ QPSK PAYLOAD (scrambler) ]
% Shaping: RRC (sps=8, rolloff=0.35, span=10)
% Requiere: save_sc16q11()
%
% Uso típico:
%   tx_img_qpsk_stable('foto.jpeg','transmision_qpsk.sc16q11');
% o simplemente:
%   tx_img_qpsk_stable;  % pedirá la ruta por teclado

%% ===== Parámetros PHY =====
sps     = 8;
rolloff = 0.35;
span    = 10;
phase0  = pi/4;         % QPSK offset (debe coincidir con RX)

%% ===== Entrada por teclado (si no viene) =====
if nargin < 1 || isempty(imgPath)
    imgPath = strtrim(input('Ruta de la imagen (.jpg/.jpeg/.png): ','s'));
    if ~exist(imgPath,'file'), error('No existe: %s',imgPath); end
end
if nargin < 2 || isempty(outSc16)
    outSc16 = 'transmision_qpsk.sc16q11';
end

%% ===== Leer imagen y normalizar (uint8, 1 ó 3 canales) =====
[img, H, W, C, meta] = load_image_uint8_1or3(imgPath);
bytesImg = typecast(img(:),'uint8');
numBytes = numel(bytesImg);
numBits  = numBytes*8;

%% ===== Cabecera (MSB-first) + CRC16 =====
hdr_bits = [ ...
    de2bi(uint16(H),16,'left-msb'), ...
    de2bi(uint16(W),16,'left-msb'), ...
    de2bi(uint8(C), 8,'left-msb')  ...
];                                  % 40 bits
crcH = crc16(hdr_bits);             % 16 bits
hdr_bits = [hdr_bits, crcH];        % 56 bits total
HDR_SYMS = ceil(numel(hdr_bits)/2); % QPSK -> 2 bits/símbolo

%% ===== PAYLOAD bits (MSB-first) + scrambler 7-bit =====
pay_bits = reshape(de2bi(bytesImg,8,'left-msb').',[],1).';   % fila
pay_bits = lfsr_scramble(pay_bits, 127);                     % 1+x^4+x^7
PAY_SYMS = ceil(numel(pay_bits)/2);

%% ===== PREÁMBULO: Barker-13×2 (BPSK) =====
lenBark = 13; repBark = 2;
barker  = comm.BarkerCode('Length',lenBark,'SamplesPerFrame',lenBark);
barkBits= (1 + barker())/2;
pre_sym = pskmod(repmat(barkBits,repBark,1), 2, 0);          % +1/-1
PRE_SYMS = numel(pre_sym);

%% ===== ENTRENAMIENTO QPSK: 64 símbolos conocidos =====
pat16     = pskmod([0;1;2;3; 1;0;3;2; 2;3;0;1; 3;2;1;0], 4, phase0, 'gray');
train_sym = repmat(pat16,4,1);                               % 64
TRN_SYMS  = numel(train_sym);

%% ===== QPSK(π/4, Gray): header + payload =====
map2sym = @(bits) pskmod(bi2de(reshape(bits,2,[]).','left-msb'), 4, phase0, 'gray');
sy_hdr  = map2sym(hdr_bits);
sy_pay  = map2sym(pay_bits);

all_syms = [pre_sym; train_sym; sy_hdr; sy_pay];             % 1 sps
TOT_SYMS = numel(all_syms);

%% ===== Filtro RRC TX (upsample sps) =====
txFilt = comm.RaisedCosineTransmitFilter( ...
    'RolloffFactor', rolloff, ...
    'FilterSpanInSymbols', span, ...
    'OutputSamplesPerSymbol', sps);
txSig = txFilt([all_syms; zeros(span,1)]);
txSig = txSig ./ max(abs(txSig));                            % normaliza
TOT_SAMPS = numel(txSig);

%% ===== Guardar salida =====
save_sc16q11(outSc16, txSig);

%% ===== Logs útiles =====
fprintf('\n=== TX listo ===\n');
fprintf('Archivo imagen: %s  | Formato reportado: %s\n', imgPath, meta.Format);
fprintf('Dimensiones: %dx%dx%d  | Pixels: %.3f MPix\n', H, W, C, H*W/1e6);
fprintf('Payload: %d bytes (%.2f kB) = %d bits\n', numBytes, numBytes/1024, numBits);
fprintf('Símbolos: PRE=%d  TRN=%d  HDR=%d  PAY=%d  | TOTAL=%d\n', PRE_SYMS,TRN_SYMS,HDR_SYMS,PAY_SYMS,TOT_SYMS);
fprintf('RRC: sps=%d, rolloff=%.2f, span=%d  | Muestras totales: %d\n', sps,rolloff,span,TOT_SAMPS);
fprintf('Salida: %s\n\n', outSc16);

%% ===== Figuras: imagen + constelaciones =====
try
    % 1) Imagen a transmitir
    figure('Name','Imagen TX (normalizada)'); imshow(img);
    title(sprintf('TX: %dx%dx%d  (%s)',H,W,C,upper(meta.Format)));

    % 2) Mapeo BPSK del preámbulo
    figure('Name','Constelación BPSK (preámbulo)');
    plot(real(pre_sym), imag(pre_sym), '.'); axis equal; grid on;
    title('BPSK preámbulo (Barker-13×2)');

    % 3) Mapeo QPSK (training + header + payload)
    figure('Name','Constelación QPSK (training+header+payload)');
    plot(real([train_sym; sy_hdr(1:min(end,200)); sy_pay(1:min(end,4000))]), ...
         imag([train_sym; sy_hdr(1:min(end,200)); sy_pay(1:min(end,4000))]), '.');
    axis equal; grid on; title('QPSK π/4, Gray (muestra)');
catch, end
end

%% ==================== Helpers locales ====================

function [img, H, W, C, meta] = load_image_uint8_1or3(imgPath)
% Lee PNG/JPG/JPEG y devuelve uint8 con 1 (gris) o 3 (RGB) canales.
% - Indexado -> RGB
% - RGBA -> RGB (descarta alfa)
% - 16-bit / double / single / logical -> uint8
% - Aplica EXIF Orientation (si existe)

meta = imfinfo(imgPath);
[A, map, alpha] = imread(imgPath);

% Indexado -> RGB (double [0..1])
if ~isempty(map)
    A = ind2rgb(A, map);
end

% Quitar alfa si existe (RGBA -> RGB)
if ndims(A) == 3 && size(A,3) == 4
    A = A(:,:,1:3);
end

% Convertir a uint8 si no lo es
if ~isa(A,'uint8')
    A = im2uint8(A);
end

% Aplicar EXIF Orientation
if isfield(meta,'Orientation')
    switch meta.Orientation
        case 1  % normal
        case 2, A = fliplr(A);
        case 3, A = rot90(A,2);
        case 4, A = flipud(A);
        case 5, A = rot90(flipud(A),1);
        case 6, A = rot90(A,-1);   % 90° CW
        case 7, A = rot90(flipud(A),-1);
        case 8, A = rot90(A,1);    % 90° CCW
    end
end

img = A;                   % 2D -> gris, 3D -> RGB
H = size(img,1); W = size(img,2);
C = size(img,3); if isempty(C), C = 1; end
end

function bitsOut = lfsr_scramble(bitsIn, seed)
% Scrambler 7-bit: s(n) = b(n) XOR (s4 XOR s7)  (polinomio 1 + x^4 + x^7)
s = de2bi(seed,7,'left-msb'); s = s(:).';
bitsOut = false(size(bitsIn));
for n = 1:numel(bitsIn)
    fb = xor(s(4), s(7));
    bitsOut(n) = xor(bitsIn(n), fb);
    s = [fb s(1:6)];
end
end

function crc = crc16(bits)
% CRC-16-IBM (poly 0xA001, init 0xFFFF), bits MSB-first
crcReg = uint16(hex2dec('FFFF'));
for i = 1:numel(bits)
    inbit  = logical(bits(i));
    xorbit = bitget(crcReg,1) ~= inbit;
    crcReg = bitshift(crcReg,-1);
    if xorbit, crcReg = bitxor(crcReg, hex2dec('A001')); end
end
crc = de2bi(crcReg,16,'left-msb');
end
