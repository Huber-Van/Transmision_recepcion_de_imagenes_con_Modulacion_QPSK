# Transmision_recepcion_de_imagenes_con_Modulacion_QPSK
# Transmisión y recepción de imágenes con QPSK (MATLAB)

Este proyecto implementa un enlace digital completo de **transmisión y recepción de imágenes** usando:

- **Modulación QPSK** con offset **π/4** y mapeo **Gray**  
- **Filtrado Raised Cosine (RRC)** en transmisión y recepción  
- **Sincronización automática en el receptor** (timing, frecuencia y fase)  
- Reconstrucción de la imagen **solo a partir de muestras I/Q** almacenadas en un archivo `.sc16q11`

- Lenguaje: **MATLAB**
- Dominio: **Comunicaciones digitales / SDR**
- Scripts principales:
  - Transmisor: `tx_img_qpsk_simple.m`
  - Receptor: `rx_img_qpsk_simple.m`

En resumen: se toma una imagen (`.jpg/.jpeg/.png`), se construye un frame QPSK robusto, se genera un archivo de muestras complejas (`.sc16q11`) y, en el receptor, se recupera la imagen únicamente a partir de esas muestras I/Q.

---

## 1. Descripción general del flujo

Cadena completa TX/RX:

```text
Imagen → Bits → Scrambler → Frame QPSK (PRE + TRAIN + HEADER + PAYLOAD)
      → Filtro RRC → Archivo IQ (.sc16q11)
      → (canal / SDR)
      → IQ recibido → RRC → Symbol Sync
      → Detección Barker → Estimación CFO/fase
      → Training QPSK → Búsqueda HEADER (rot/slip + CRC)
      → PLL QPSK → Descrambler → Reconstrucción de imagen

