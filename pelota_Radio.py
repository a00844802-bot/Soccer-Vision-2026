import sensor, image, time
from pyb import UART, LED

# --- CONFIGURACIÓN DE CÁMARA ---
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

# --- CONFIGURACIÓN UART PARA ARDUINO ---
uart = UART(3, 9600, timeout_char=1000)
time.sleep(2)
print("OpenMV iniciada")

# --- LEDs INDICADORES ---
led_rojo = LED(1)    # LED rojo
led_verde = LED(2)   # LED verde
led_azul = LED(3)    # LED azul

# --- UMBRAL DE COLOR ROJO EN LAB ---
# Formato: (L Min, L Max, A Min, A Max, B Min, B Max)
# Para rojo: A alto (positivo indica rojo)
red_threshold = (30, 100, 15, 127, 15, 127)

# --- PARÁMETROS ---
ANCHO = 320
ALTO = 240
CENTRO_X = ANCHO // 2
CENTRO_Y = ALTO // 2

ZONA_MUERTA = 50          # Tolerancia en píxeles
AREA_MINIMA = 100         # Área mínima del blob
RADIO_MINIMO = 20         # Radio mínimo para acercarse
PROPORCION_LINEA = 0.7    # Para detectar líneas horizontales

clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()
    
    pelota_detectada = False
    linea_detectada = False
    
    # Buscar blobs rojos
    blobs = img.find_blobs([red_threshold], 
                          pixels_threshold=200, 
                          area_threshold=AREA_MINIMA,
                          merge=True)
    
    if blobs:
        # Ordenar por área (mayor primero)
        blobs = sorted(blobs, key=lambda b: b.area(), reverse=True)
        
        for blob in blobs:
            # Calcular aspect ratio
            aspect_ratio = blob.w() / float(blob.h()) if blob.h() > 0 else 0
            
            # --- DETECCIÓN DE LÍNEA HORIZONTAL ---
            if aspect_ratio > 4.0 and blob.w() > ANCHO * PROPORCION_LINEA:
                print("LÍNEA DETECTADA - DETENER")
                img.draw_rectangle(blob.rect(), color=(255, 0, 0), thickness=2)
                img.draw_string(10, 10, "LINEA!", color=(255, 0, 0), scale=2)
                
                linea_detectada = True
                led_rojo.on()
                led_verde.off()
                led_azul.off()
                
                uart.write('S')  # Detener
                break
            
            # --- DETECCIÓN DE PELOTA ---
            # Verificar circularidad (compacidad)
            circularidad = blob.compactness()
            
            if circularidad > 0.5:  # Objeto circular
                pelota_detectada = True
                
                # Dibujar detección
                img.draw_rectangle(blob.rect(), color=(0, 255, 0))
                img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))
                
                # Calcular desplazamiento
                desp_x = blob.cx() - CENTRO_X
                desp_y = blob.cy() - CENTRO_Y
                
                # Estimar distancia por el radio
                radio = (blob.w() + blob.h()) / 4
                
                comando = ""
                
                # Lógica de control
                if abs(desp_x) > ZONA_MUERTA:
                    if desp_x > 0:
                        comando = "D"  # Girar derecha
                        print(">>> DERECHA | Desp: %d" % desp_x)
                        led_azul.on()
                        led_verde.off()
                    else:
                        comando = "I"  # Girar izquierda
                        print("<<< IZQUIERDA | Desp: %d" % desp_x)
                        led_verde.on()
                        led_azul.off()
                        
                elif radio < RADIO_MINIMO:
                    comando = "A"  # Avanzar
                    print("^^^ AVANZAR | Radio: %d" % radio)
                    led_verde.on()
                    led_azul.on()
                    
                else:
                    comando = "S"  # Centrado
                    print("=== CENTRADO | Radio: %d ===" % radio)
                    led_verde.on()
                    led_azul.off()
                
                # Mostrar info en pantalla
                img.draw_string(10, 10, "R:%d X:%d" % (int(radio), desp_x), 
                              color=(255, 255, 255), scale=1)
                
                # Enviar comando
                if comando:
                    uart.write(comando)
                
                led_rojo.off()
                break  # Solo procesar la pelota más grande
    
    # --- SI NO SE DETECTA NADA ---
    if not pelota_detectada and not linea_detectada:
        img.draw_string(10, 10, "BUSCANDO...", color=(255, 255, 0), scale=2)
        print("Buscando pelota...")
        uart.write('S')  # Detener mientras busca
        
        led_rojo.off()
        led_verde.off()
        led_azul.toggle()  # Parpadeo azul
    
    # Mostrar FPS
    print("FPS: %d" % clock.fps())