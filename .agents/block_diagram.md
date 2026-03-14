# Sistem Blok Diyagramı

```mermaid
block-beta
columns 5

block:SENSORS["SENSÖRLER"]:2
  ZED["ZED 2 Stereo Kamera\n(Sabit, 110° HFOV)\nUSB 3.0"]
  LOG["Logitech C920\n(Pan-Tilt Üzerinde, 78°)\nUSB + DSHOW"]
  LIDAR["Benewake TF03\n1D LiDAR\nUART COM3"]
end

space:3

block:PROCESS["YAZILIM İŞLEME (Python 3.10)"]:5
  YOLO["YOLOv8\nTensorRT Engine\nbatch=2"]
  BT["ByteTrack\nÇoklu Nesne Takip"]
  DEPTH["Stereo Derinlik\n+ Yaklaşma Hızı"]
  PD_CTRL["PD Tabanlı\nGörsel Servo\nKontrol"]
  COLOR["LAB Renk\nSınıflandırma\n(Stage 3)"]
end

block:ACTUATORS["EYLEYICILER"]:2
  ARD["Arduino Mega\nUART COM4\n115200 baud"]
  PANTILT["Pan-Tilt\nMekanizması\n(Step Motorlar)"]
  LASER["Lazer Modül\nPin 9"]
end

space:3

block:UI["KULLANICI ARAYÜZÜ"]:5
  CSHARP["C# WinForms\n(.NET 6.0)"]
  MMF["Memory-Mapped\nFile (MMF)\nShared Memory"]
end
```

---

## Veri Akış Blok Diyagramı

```mermaid
flowchart LR
    subgraph SENSORS["Sensörler"]
        ZED["ZED 2\n(Sabit)"]
        LOG["Logitech C920\n(Pan-Tilt)"]
        LIDAR["TF03 LiDAR"]
    end

    subgraph PYTHON["Python Backend"]
        subgraph DETECT["Algılama"]
            YOLO["YOLOv8\nTensorRT\nbatch=2"]
            BT["ByteTrack"]
        end
        subgraph CALC["Hesaplama"]
            DEPTH["Stereo Derinlik"]
            VZ["Yaklaşma Hızı\nvz = Δd/Δt"]
            OMEGA["Açısal Hız\nω = vx/W × FOV"]
            PRLX["Paralaks\nTelafisi"]
        end
        subgraph CTRL["Kontrol"]
            PRIO["Hedef\nÖnceliklendirme"]
            HO["Handover\nSpotter→Tracker"]
            PD["PD Kontrol\nPixel Error→Açı"]
            LAB["LAB Renk\nDost/Düşman"]
            RPIX["Kırmızı Piksel\nHedefleme"]
        end
    end

    subgraph HW["Eyleyiciler"]
        ARD["Arduino\nMega"]
        PT["Pan-Tilt\nMotorlar"]
        LAS["Lazer"]
    end

    subgraph UI["Arayüz"]
        MMF["MMF\nShared Memory"]
        CS["C# WinForms"]
    end

    %% Sensör → Algılama
    ZED -- "frame" --> YOLO
    LOG -- "frame" --> YOLO
    YOLO -- "bbox, class, conf" --> BT
    ZED -- "depth map" --> DEPTH
    LIDAR -- "9-byte paket\nCOM3" --> PRLX

    %% Algılama → Hesaplama
    BT -- "track_id, position" --> VZ
    DEPTH -- "mesafe (m)" --> VZ
    BT -- "Δposition" --> OMEGA

    %% Hesaplama → Kontrol
    VZ -- "vz, ETA" --> PRIO
    OMEGA -- "ω (°/s)" --> PD
    PRIO -- "primary target" --> HO
    HO -- "hedef koordinat" --> PD
    DEPTH -- "depth (m)" --> PRLX
    PRLX -- "düzeltilmiş hedef" --> PD

    %% Stage spesifik
    LOG -- "bbox ROI" --> LAB
    LOG -- "bbox ROI" --> RPIX

    %% Kontrol → Eyleyiciler
    PD -- "E{errX},Y{errY}\nUART COM4" --> ARD
    HO -- "P{pan},T{tilt}" --> ARD
    ARD -- "step pulse" --> PT
    ARD -- "Pin 9 HIGH" --> LAS

    %% UI haberleşme
    YOLO -- "processed_frame" --> MMF
    MMF -- "görüntü" --> CS
    CS -- "SystemMode\nFireData" --> MMF
    MMF -- "komut" --> CTRL
```

---

## Donanım Bağlantı Blok Diyagramı

```mermaid
flowchart TB
    subgraph PC["Bilgisayar (GPU + CPU)"]
        PY["Python Backend"]
        CS["C# WinForms UI"]
        PY <-- "MMF\nShared Memory" --> CS
    end

    subgraph TRIPOD["Sabit Tripod"]
        ZED["ZED 2\nStereo Kamera"]
    end

    subgraph PANTILT["Pan-Tilt Mekanizması"]
        LOG["Logitech C920"]
        LAS["Lazer Modül"]
        LID["TF03 LiDAR"]
    end

    ARD["Arduino Mega"]

    ZED -- "USB 3.0" --> PC
    LOG -- "USB 2.0" --> PC
    LID -- "UART COM3\n115200 baud" --> PC
    ARD -- "UART COM4\n115200 baud" --> PC
    ARD -- "Step/Dir\nPin 2,3,5,6" --> PANTILT
    ARD -- "Pin 9" --> LAS
