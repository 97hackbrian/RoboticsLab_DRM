import cv2
from matplotlib.pyplot import prism
import numpy
from abc import ABC, abstractmethod

import numpy as np


class img_abs(ABC):

    '''
    @abstractmethod
    def readIMG(self):
        pass
    '''
    
    @abstractmethod
    def showIMG(self):
        pass
    
    @abstractmethod
    def resize_img(self):
        pass
    
    @abstractmethod
    def rotate_image(self):
        pass
    
    @abstractmethod
    def cutHalves(self):
        pass

    @abstractmethod
    def cutQ(self):
        pass

    @abstractmethod
    def convIMGgray(self):
        pass

    @abstractmethod
    def convIMGhsv(self):
        pass

    @abstractmethod
    def save(self):
        pass

    @abstractmethod
    def retorno(self):
        pass
    
    @abstractmethod
    def Cannycontours(self):
        pass
    
    @abstractmethod
    def apply_binary_threshold(self):
        pass

    @abstractmethod
    def apply_otsu_threshold(self):
        pass

    @abstractmethod
    def apply_inverse_binary_threshold(self):
        pass

    @abstractmethod
    def apply_truncated_threshold(self):
        pass

    @abstractmethod
    def apply_tozero_threshold(self):
        pass


    @abstractmethod
    def apply_inverse_tozero_threshold(self):
        pass

    @abstractmethod
    def apply_erosion(self):
        pass

    @abstractmethod
    def apply_dilation(self):
        pass

    @abstractmethod
    def draw(self):
        pass

class video_abc(ABC):
    @abstractmethod
    def load(self):
        pass

    @abstractmethod
    def actions(self):
        pass

    @abstractmethod
    def videoPlay(self):
        pass

    @abstractmethod
    def videoStop(self):
        pass

    @abstractmethod
    def retorno(self):
        pass


class img(img_abs):
    def __init__(self,image,tipo) -> None:
        self.contador_imagenes=0
        if type(image)==str:
            #imagen="/imagenes/perrito1.png"
            self.imagen = cv2.imread(image)
            self.imagen=self.tipos(self.imagen,tipo)
            #print("dir!")
        #print("OriginalSize",img.shape)
        elif type(image)==numpy.ndarray:
            #print("np!")
            self.imagen=self.tipos(image,tipo)


    def tipos(self,img,tipo):
        
        if(tipo=="BGR"):
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # type: ignore
        elif(tipo=="HSV"):
            img = cv2.cvtColor(img, cv2.COLOR_HSV2RGB) # type: ignore
        elif(tipo=="GRAY"):
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # type: ignore
        elif(tipo=="RGB"):
            pass
        return img
    
    def showIMG(self):
        cv2.imshow('showed Instance',self.imagen) # type: ignore
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    


    def resize_img(self,width, height,retorno):
        up_points = (width, height)
        img_resize = cv2.resize(self.imagen, up_points) # type: ignore
        #print("NewSize",img_resize.shape)
        if(retorno==1):
            return img_resize
        else:
            self.imagen=img_resize
        

    
    def rotate_image(self, degrees,retorno):
        height, width = self.imagen.shape[:2]
        center = (width // 2, height // 2)
        rotation_matrix = cv2.getRotationMatrix2D(center, degrees, 1.0)
        rotated_image = cv2.warpAffine(self.imagen, rotation_matrix, (width, height))
        if(retorno==1):
            return rotated_image
        else:
            self.imagen=rotated_image
        

    def cutHalves(self):
        resized_image=self.imagen.copy()
        height, width = resized_image.shape[:2]
        half_height = height // 2
        upper_half = resized_image[:half_height, :]
        lower_half = resized_image[half_height:, :]

        half_width = width // 2
        left_half = resized_image[:, :half_width]
        right_half = resized_image[:, half_width:]
        
        return left_half,right_half,upper_half,lower_half
    
    def cutQ(self):
    # Divide the image into quadrants
        image = self.imagen.copy()
        height, width = image.shape[:2]

        if len(image.shape) == 3:  # Si la imagen es a color
            quadrant1 = image[:height // 2, :width // 2]
            quadrant2 = image[:height // 2, width // 2:]
            quadrant3 = image[height // 2:, :width // 2]
            quadrant4 = image[height // 2:, width // 2:]
        else:  # Si la imagen es en escala de grises
            quadrant1 = image[:height // 2, :width // 2]
            quadrant2 = image[:height // 2, width // 2:]
            quadrant3 = image[height // 2:, :width // 2]
            quadrant4 = image[height // 2:, width // 2:]

        return quadrant1, quadrant2, quadrant3, quadrant4

    def convIMGgray(self,retorno):
        img = cv2.cvtColor(self.imagen, cv2.COLOR_RGB2GRAY)
        
        if retorno==1:
            return img
        else:
            self.imagen=img
        

    def convIMGhsv(self,retorno):
        img = cv2.cvtColor(self.imagen, cv2.COLOR_RGB2HSV)
        if retorno==1:
            return img
        else:
            self.imagen=img
        
    
    def save(self,key,key2,same,every):
        if same ==0:
            if type(key)==str:
                while(1):
                    cv2.imshow("save?",self.imagen)
                    if cv2.waitKey(0) & 0xFF == ord(key):
                        ruta_imagen = f'Raspberry/lab8/Saves/image{self.contador_imagenes}.jpg'
                        cv2.imwrite(ruta_imagen, self.imagen)
                        print(f"Imagen guardada en {ruta_imagen}")
                        self.contador_imagenes += 1
                        cv2.destroyAllWindows()
                        break
                    elif cv2.waitKey(0) & 0xFF == ord(key2):
                        print("Guardado cancelado")
                        cv2.destroyAllWindows()
                        break
                    else:
                        print("No key")

            elif(key==0):
                ruta_imagen = f'Raspberry/lab8/Saves/image{self.contador_imagenes}.jpg'
                cv2.imwrite(ruta_imagen, self.imagen)
                print(f"Imagen guardada en {ruta_imagen}")
                self.contador_imagenes += 1
        else:
            if type(key)==str:
                while(1):
                    #cv2.imshow("save?",self.imagen)
                    if every==1:
                        ruta_imagen = f'Raspberry/lab8/Saves/image1.jpg'
                        cv2.imwrite(ruta_imagen, self.imagen)
                    else:
                        if cv2.waitKey(0) & 0xFF == ord(key) :
                            ruta_imagen = f'Raspberry/lab8/Saves/image1.jpg'
                            cv2.imwrite(ruta_imagen, self.imagen)
                            print(f"Imagen guardada en {ruta_imagen}")
                            self.contador_imagenes += 1
                            cv2.destroyAllWindows()
                            break
                        
                        else:
                            
                            print("No key")

            elif(key==0):
                #print("aqui")
                ruta_imagen = f'Raspberry/lab8/Saves/image1.jpg'
                cv2.imwrite(ruta_imagen, self.imagen)
                print(f"Imagen guardada en {ruta_imagen}")
                self.contador_imagenes += 1

    def retorno(self):
        return self.imagen
    
    def Cannycontours(self,u1,u2,b1,b2,retorno):
        img=self.convIMGgray(1)
        img = cv2.GaussianBlur(self.imagen,(b1,b2),0)
        Canny = cv2.Canny(img, u1, u2)

        if retorno==1:
            return Canny
        else:
            self.imagen= Canny
    
    def apply_binary_threshold(self, threshold_value,retorno):
        _, binary_threshold = cv2.threshold(self.imagen, threshold_value, 255, cv2.THRESH_BINARY)
        if retorno==1:
            return binary_threshold
        else:
            
            self.imagen=binary_threshold


    
    def apply_otsu_threshold(self, threshold_value,retorno):
        _, otsu_threshold = cv2.threshold(self.imagen, threshold_value, 255, cv2.THRESH_OTSU)
        if retorno==1:
            return otsu_threshold
        else:
            self.imagen=otsu_threshold

    def apply_inverse_binary_threshold(self, threshold_value,retorno):
        _, inverse_binary_threshold = cv2.threshold(self.imagen, threshold_value, 255, cv2.THRESH_BINARY_INV)
        if retorno==1:
            return inverse_binary_threshold
        else:
            self.imagen=inverse_binary_threshold

    def apply_truncated_threshold(self, threshold_value,retorno):
        _, truncated_threshold = cv2.threshold(self.imagen, threshold_value, 255, cv2.THRESH_TRUNC)
        if retorno==1:
            return truncated_threshold
        else:
            self.imagen=truncated_threshold

    def apply_tozero_threshold(self, threshold_value,retorno):
        _, tozero_threshold = cv2.threshold(self.imagen, threshold_value, 255, cv2.THRESH_TOZERO)
        if retorno==1:
            return tozero_threshold
        else:
            self.imagen=tozero_threshold

    def apply_inverse_tozero_threshold(self, threshold_value,retorno):
        _, inverse_tozero_threshold = cv2.threshold(self.imagen, threshold_value, 255, cv2.THRESH_TOZERO_INV)
        if retorno==1:
            return inverse_tozero_threshold
        else:
            self.imagen=inverse_tozero_threshold

    def apply_erosion(self, threshold_value, erosion_iterations,retorno):
        binary_threshold = self.apply_binary_threshold(threshold_value,1) # type: ignore
        kernel = np.ones((2, 2), np.uint8)
        erosion = cv2.erode(binary_threshold, kernel, iterations=erosion_iterations) # type: ignore
        if retorno==1:
            return erosion
        else:
            self.imagen=erosion

    def apply_dilation(self, threshold_value, dilation_iterations,retorno):
        binary_threshold = self.apply_binary_threshold(threshold_value,1) # type: ignore
        kernel = np.ones((2, 2), np.uint8)
        dilation = cv2.dilate(binary_threshold, kernel, iterations=dilation_iterations)
        if retorno==1:
            return dilation
        else:
            self.imagen=dilation
    

    def draw(self,val,retorno):
        contour_image = self.imagen.copy()
        gray=self.convIMGgray(1)
        _, thresholding=cv2.threshold(gray, val, 255, cv2.THRESH_OTSU) # type: ignore
        contours, _ = cv2.findContours(thresholding, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        cv2.drawContours(contour_image, contours, -1, (0, 0, 255), 1)
        if retorno==1:
            return contour_image,contours
        else:
            self.imagen=contour_image

    def split_channels(self):
        if len(self.imagen.shape) == 3:  # Solo para imágenes a color
            blue_channel = self.imagen[:, :, 0]
            green_channel = self.imagen[:, :, 1]
            red_channel = self.imagen[:, :, 2]
            return blue_channel, green_channel, red_channel
        else:
            print("La imagen no es a color. No se pueden dividir los canales.")
            return None



def showIMG(imagenes):
    c=0
    for x in imagenes:
        cv2.imshow(str(c),x) # type: ignore
        c=c+1
    cv2.waitKey(0)
    cv2.destroyAllWindows()






import numpy as np
from multiprocessing import Process
class video(video_abc,img):
    def __init__(self) -> None:
        self.frames=[]
    def load(self, video_source):
        if isinstance(video_source, str):
            # Si se proporciona una cadena (ruta de archivo), cargamos el archivo de video
            print("Cargando video desde:", video_source)
            vid = cv2.VideoCapture(video_source)
            if not vid.isOpened():
                print("Error al abrir el archivo de video")
                return

            while True:
                ret, frame = vid.read()
                if not ret:
                    break
                self.frames.append(frame)

            vid.release()
            print("Video cargado. Total de frames:", len(self.frames))
        elif isinstance(video_source, list):
            # Si se proporciona una lista de frames, usamos esos frames como el video
            print("Cargando video desde una lista de frames.")
            self.frames = video_source
            print("Video cargado. Total de frames:", len(self.frames))
        else:
            print("Fuente de video no válida. Debe ser una ruta de archivo o una lista de frames.")

    def actions(self):
        pass

    def rotate(self,angle, retorno):
        frames_rotados = []
        for frame in self.frames:
            frame_obj = img(frame, tipo="RGB")
            frame_obj.rotate_image(angle, retorno=0)
            frames_rotados.append(frame_obj.imagen)

        if retorno==1:
            return frames_rotados
        else:
            self.frames = frames_rotados

    def resize(self,width,height,retorno):
        frames_resized = []
        for frame in self.frames:
            frame_obj = img(frame, tipo="RGB")
            frame_obj.resize_img(width,height,0)
            frames_resized.append(frame_obj.imagen)
        if retorno==1:
            return frames_resized
        else:
            self.frames = frames_resized

    def canny(self,u1,u2,b1,b2,retorno):
        frames_canny=[]
        for frame in self.frames:
            frame_obj = img(frame, tipo="RGB")
            frame_obj.Cannycontours(u1,u2,b1,b2,0)
            frames_canny.append(frame_obj.imagen)
        if retorno==1:
            return frames_canny
        else:
            self.frames = frames_canny


    def halves(self, retorno):
        left_halves, right_halves, upper_halves, lower_halves = [], [], [], []
        
        for frame in self.frames:
            frame_obj = img(frame, tipo="RGB")
            left_half, right_half, upper_half, lower_half = frame_obj.cutHalves()

            left_halves.append(left_half)
            right_halves.append(right_half)
            upper_halves.append(upper_half)
            lower_halves.append(lower_half)

        if retorno == 1:
            return left_halves, right_halves, upper_halves, lower_halves
        else:
            self.frames = [left_halves, right_halves, upper_halves, lower_halves]
    def quadrants(self, retorno):
        quadrant1_frames, quadrant2_frames, quadrant3_frames, quadrant4_frames = [], [], [], []

        for frame in self.frames:
            frame_obj = img(frame, tipo="RGB")
            quadrant1, quadrant2, quadrant3, quadrant4 = frame_obj.cutQ()
            quadrant1_frames.append(quadrant1)
            quadrant2_frames.append(quadrant2)
            quadrant3_frames.append(quadrant3)
            quadrant4_frames.append(quadrant4)

        if retorno == 1:
            return quadrant1_frames, quadrant2_frames, quadrant3_frames, quadrant4_frames
        else:
            self.frames = [quadrant1_frames, quadrant2_frames, quadrant3_frames, quadrant4_frames]
    
    
    def change_to_color_channels(self):
        frames_color_channels = []
        for frame in self.frames:
            frame_obj = img(frame, tipo="RGB")
            _, green_channel, red_channel = frame_obj.split_channels()  # Obtenemos los canales de color
            frame_color = cv2.merge((blue_channel, green_channel, red_channel))  # Combinamos los canales
            frames_color_channels.append(frame_color)
        return frames_color_channels

    '''
    def videoPlay(self):
        print("Reproduciendo video...")
        for frame in self.frames:
            cv2.imshow('Video', frame)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                print("Pausado y cerrado")
                break
        print("Video Finalizado")
        cv2.destroyAllWindows()
    '''

    def videoPlay(self):
        print("Reproduciendo video...")
        for frame in self.frames:
            cv2.imshow('Video', frame)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                print("Pausado y cerrado")
                break
        print("Video Finalizado")
        cv2.destroyAllWindows()


    def videoStop(self):
        pass

    def retorno(self):
        pass

def videoPlayer(frame, window_name):
    for frames in frame:
        cv2.imshow(window_name, frames)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

def videosPlays(self, video_list):
    processes = []

    for i, video_frames in enumerate(video_list):
        window_name = f"Video {i+1}"
        p = Process(target=videoPlayer, args=(video_frames, window_name))
        p.start()
        processes.append(p)

    for p in processes:
        p.join()

    for p in processes:
        p.terminate()

    cv2.destroyAllWindows()



class Camara:
    def __init__(self, cam_source=0) -> None:
        super().__init__()
        self.cam_source = cam_source
        self.capture = cv2.VideoCapture(cam_source)
        if not self.capture.isOpened():
            print("Error al abrir la cámara")
        self.camera_window_name = "Cámara"

    def load(self):
        pass  # La cámara se carga en el constructor, no es necesario cargarla de nuevo

    def actions(self):
        pass  # Puedes agregar acciones específicas de la cámara aquí si es necesario

    def videoPlay(self):
        print("Reproduciendo desde la cámara...")
        cv2.namedWindow(self.camera_window_name, cv2.WINDOW_NORMAL)  # Crea una ventana
        while True:

            ret, frame = self.capture.read()
            if not ret:
                print("No se pudo obtener un fotograma de la cámara")
                break
            cv2.imshow(self.camera_window_name, frame)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                print("Pausado y cerrado")
                break
        print("Video de cámara finalizado")
        cv2.destroyWindow(self.camera_window_name)  # Cierra la ventana al final

    def videoRotate(self,clase,angle,color):
        print("Reproduciendo desde la cámara...")
        cv2.namedWindow(self.camera_window_name, cv2.WINDOW_NORMAL)  # Crea una ventana
        while True:

            ret, frame = self.capture.read()
            frame=clase(frame,color)
            frame.rotate_image(angle,0)
            if not ret:
                print("No se pudo obtener un fotograma de la cámara")
                break
            cv2.imshow(self.camera_window_name, frame.imagen)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                print("Pausado y cerrado")
                break
        print("Video de cámara finalizado")
        cv2.destroyWindow(self.camera_window_name)  # Cierra la ventana al final


    def videoColor(self,clase,scale,canal):
        print("Reproduciendo desde la cámara...")
        cv2.namedWindow(self.camera_window_name, cv2.WINDOW_NORMAL)  # Crea una ventana
        while True:

            ret, frame = self.capture.read()
            frame=clase(frame,scale)
            if canal==1:
                frame,_,_=frame.split_channels()
            elif canal==2:
                _,frame,_=frame.split_channels()
            elif canal==3:
                _,_,frame=frame.split_channels()
            if not ret:
                print("No se pudo obtener un fotograma de la cámara")
                break
            cv2.imshow(self.camera_window_name, frame)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                print("Pausado y cerrado")
                break
        print("Video de cámara finalizado")
        cv2.destroyWindow(self.camera_window_name)  # Cierra la ventana al final

    def change_to_color_channel(self, channel='RGB'):
        frames_color_channel = []
        for frame in self.frames:
            frame_obj = img(frame, tipo="RGB")
            if channel == 'RGB':
                frame_color_channel = frame  # Mostrar el canal RGB original
            elif channel == 'Red':
                _, _, frame_color_channel = frame_obj.split_channels()  # Mostrar solo el canal Rojo
            elif channel == 'Green':
                _, frame_color_channel, _ = frame_obj.split_channels()  # Mostrar solo el canal Verde
            elif channel == 'Blue':
                frame_color_channel, _, _ = frame_obj.split_channels()  # Mostrar solo el canal Azul
            frames_color_channel.append(frame_color_channel)
        self.capture=frames_color_channel
        return frames_color_channel
    
    def saveCameracapture(self,clase,key,every):
        print("Esperando a guardar")
        cv2.namedWindow(self.camera_window_name, cv2.WINDOW_NORMAL)  # Crea una ventana
        while True:

            ret, frame = self.capture.read()
            frame=clase(frame,"RGB")
            frame.save(key,"q",1,every)
            if not ret:
                print("No se pudo obtener un fotograma de la cámara")
                break
            cv2.imshow("self.camera_window_name", frame.imagen)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                print("Pausado y cerrado")
                break
            
        print("Video de cámara finalizado")
        cv2.destroyWindow(self.camera_window_name)  # Cierra la ventana al final


    def videoCanny(self,clase,u1,u2,b1,b2):
        print("Canny desde la cámara...")
        cv2.namedWindow(self.camera_window_name, cv2.WINDOW_NORMAL)  # Crea una ventana
        while True:

            ret, frame = self.capture.read()
            frame=clase(frame,"RGB")
            frame.Cannycontours(u1,u2,b1,b2,0)
            if not ret:
                print("No se pudo obtener un fotograma de la cámara")
                break
            cv2.imshow(self.camera_window_name, frame.imagen)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                print("Pausado y cerrado")
                break
        print("Video de cámara finalizado")
        cv2.destroyWindow(self.camera_window_name)  # Cierra la ventana al final
        

    def DetecColor(self,clase,motor):
        print("Detecting color...")
        cv2.namedWindow(self.camera_window_name, cv2.WINDOW_NORMAL)
        while True:
            ret, frame = self.capture.read()
            frame_rgb=clase(frame,"BGR")
            frame_hsv=frame_rgb.convIMGhsv(1)

            h=frame_hsv[:, :, 0]
            h=int(np.mean(h))

            color = "N/A"

            if h >= 100:
                color = "Rojo"
                motor.left(0)
            elif h > 40 and h<65:
                color = "Amarillo"
                motor.left(25)
            elif h >= 65 and h<100:
                color = "Verde"
                motor.left(100)
            print(h,"  Color:", color)
            if not ret:
                print("No se pudo obtener un fotograma de la cámara")
                break
            cv2.imshow(self.camera_window_name, frame)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                print("Paused and closed")
                break
        print("Camera video finished")
        cv2.destroyWindow(self.camera_window_name)

    def videoStop(self):
        self.capture.release()

    def is_open(self):
        return self.capture is not None and self.capture.isOpened()

    def track(self):###aqui###
        if not self.is_open():
            print("La cámara no se ha cargado correctamente.")
            return

        #serial = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
        #serial.reset_input_buffer()
        #serial.reset_output_buffer()

        background_subtractor = cv2.createBackgroundSubtractorMOG2()

        while True:
            ret, frame = self.capture.read()
            frame2=img(frame,"RGB")
            frame=frame2.resize_img(500,300,1)
            if not ret:
                print("No se pudo obtener un fotograma de la cámara")
                break

            foreground_mask = background_subtractor.apply(frame)
            _, thresh = cv2.threshold(foreground_mask, 200, 255, cv2.THRESH_BINARY)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (20, 20))
            processed_foreground = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

            contours, _ = cv2.findContours(processed_foreground, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(frame, contours, -1, (255, 0, 0), 2)
            num_objects = len(contours)

            if num_objects > 0:
                if 1 <= num_objects <= 10:
                    #serial.write("2\n".encode())
                    print("UNO")
                else:
                    #serial.write("4\n".encode())
                    print("MAS DE UNO")
            else:
                print("SIN MOVIMIENTO")
                #serial.write("0\n".encode())

            cv2.imshow("track", frame)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                print("Pausado y cerrado")
                break

        print("Detección de movimiento finalizada")
        cv2.destroyWindow(self.camera_window_name)
        self.videoStop()
        

    def retorno(self):
        pass  # Puedes agregar la lógica de retorno específica de la cámara aquí si es necesario
