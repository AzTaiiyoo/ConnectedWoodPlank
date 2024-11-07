import asyncio
import struct
from bleak import BleakScanner, BleakClient
import logging

# Configuration du logging pour un affichage clair
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# UUIDs du service et des caractéristiques
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CAPACITIVE_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
STRAIN_GAUGE_UUID = "cc54f4ce-1037-4b73-9e5a-cdcd53e85145"
PIEZO_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a9"

class BLETestReceiver:
    def __init__(self):
        self.NUM_CAPACITIVE = 16
        self.NUM_STRAIN = 4
        self.NUM_PIEZO = 4

    def parse_capacitive(self, sender, data):
        """Affiche les données des capteurs capacitifs"""
        try:
            if data[0] != ord('<') or data[-1] != ord('>'):
                logging.warning("Marqueurs invalides pour les données capacitives")
                logging.warning(f"Données reçues (hex): {data.hex()}")
                return

            values = []
            for i in range(self.NUM_CAPACITIVE):
                value = struct.unpack_from('<H', data, offset=1+i*2)[0]
                values.append(value)
            
            logging.info(f"Données capacitives: {values}")
            logging.debug(f"Données brutes (hex): {data.hex()}")
        except Exception as e:
            logging.error(f"Erreur parsing capacitif: {str(e)}")
            logging.error(f"Données brutes (hex): {data.hex()}")

    def parse_strain_gauge(self, sender, data):
        """Affiche les données des jauges de contrainte"""
        try:
            if data[0] != ord('(') or data[-1] != ord(')'):
                logging.warning("Marqueurs invalides pour les jauges de contrainte")
                logging.warning(f"Données reçues (hex): {data.hex()}")
                return

            values = [data[i+1] for i in range(self.NUM_STRAIN)]
            logging.info(f"Données des jauges: {values}")
            logging.debug(f"Données brutes (hex): {data.hex()}")
        except Exception as e:
            logging.error(f"Erreur parsing jauge: {str(e)}")
            logging.error(f"Données brutes (hex): {data.hex()}")

    def parse_piezo(self, sender, data):
        """Affiche les données des capteurs piézoélectriques"""
        try:
            if (data[0] != ord('-') or data[1] != ord('>') or 
                data[-2] != ord('<') or data[-1] != ord('-')):
                logging.warning("Marqueurs invalides pour les données piézo")
                logging.warning(f"Données reçues (hex): {data.hex()}")
                return

            values = []
            for i in range(self.NUM_PIEZO):
                value = struct.unpack_from('>H', data, offset=2+i*2)[0]
                values.append(value)
            
            logging.info(f"Données piézo: {values}")
            logging.debug(f"Données brutes (hex): {data.hex()}")
        except Exception as e:
            logging.error(f"Erreur parsing piézo: {str(e)}")
            logging.error(f"Données brutes (hex): {data.hex()}")

    async def run(self):
        """Boucle principale de réception des données"""
        try:
            # Recherche de l'appareil
            logging.info("Recherche de l'appareil...")
            device = await BleakScanner.find_device_by_filter(
                lambda d, ad: SERVICE_UUID.lower() in (ad.service_uuids if ad and ad.service_uuids else [])
            )

            if not device:
                logging.error(f"Aucun appareil trouvé avec l'UUID {SERVICE_UUID}")
                return

            logging.info(f"Appareil trouvé: {device.name}")

            # Connexion et écoute des notifications
            async with BleakClient(device) as client:
                logging.info(f"Connecté à: {device.name}")

                # Activation des notifications pour chaque caractéristique
                await client.start_notify(CAPACITIVE_UUID, self.parse_capacitive)
                await client.start_notify(STRAIN_GAUGE_UUID, self.parse_strain_gauge)
                await client.start_notify(PIEZO_UUID, self.parse_piezo)

                logging.info("En attente de données... (Ctrl+C pour arrêter)")
                
                # Boucle infinie jusqu'à interruption
                while True:
                    await asyncio.sleep(1)

        except KeyboardInterrupt:
            logging.info("Arrêt demandé par l'utilisateur")
        except Exception as e:
            logging.error(f"Erreur de connexion: {str(e)}")

if __name__ == "__main__":
    receiver = BLETestReceiver()
    asyncio.run(receiver.run())