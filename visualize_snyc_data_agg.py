import pickle
import numpy as np
import cv2
from src.center_image_undistorted import undistorted_image_center
from src.radar_to_cam_trans import *
import os
import time
import itertools

def point_color(r):
    """
    0-200: green
    252-500: yellow
    500-750: orange
    750-: red
    """

    if r <= 250:
        return (0, 255, 0)
    elif r <= 500:
        return (0, 255, 255)
    elif r <= 750:
        return (0, 165, 255)
    else:
        return (0, 0, 255)

indexes_arr = np.load("theta_indexes_1.npy")

radar_data_agg = {index: ([], []) for index in indexes_arr.astype(np.float32)}

# Erstelle ein leeres Bild für das Radar 
target_height = 1080 # Höhe des Bildes
radar_image = np.zeros((target_height, target_height), dtype=np.uint8)


radar_background = np.zeros((target_height, target_height), dtype=np.uint8)

# Definiere die Offsets für das 3x3 Quadrat
offsets = np.array([-1, 0, 1])

# Erzeuge ein Meshgrid für die Verschiebungen
dx, dy = np.meshgrid(offsets, offsets)

# Kameramatrix und new Kameramatrix
camera_matrix_center = np.array([
        [1700.216265, 0.000000, 661.247822],
        [0.000000, 1700.105734, 514.219839],
        [0.000000, 0.000000, 1.000000]
    ], dtype=np.float32)

D = np.array([-0.259543, 0.186944, 0.001501, 0.003522, 0.000000], dtype=np.float32)

h, w = 1024, 1280
K_new, roi = cv2.getOptimalNewCameraMatrix(camera_matrix_center, D, (w, h), 1, (w, h))


# Maximale und Nominale Range für den Radar
MAX_RANGE = 977
NOMINAL_RANGE = 500


def clear_radar_image(radar_image, unique_thetas, length):
    center_x = radar_image.shape[1] // 2
    center_y = radar_image.shape[0] // 2
    
    # Erstelle ein Array für die Radien
    r = np.arange(length)
    
    # print(unique_thetas)
    
    # Erstelle ein 2D-Gitter aus Radien und Winkeln
    R, Theta = np.meshgrid(r, unique_thetas)

    
    # Berechne die x- und y-Koordinaten für alle Punkte im Gitter
    x = (center_x + R * np.cos(Theta)).astype(int)
    y = (center_y - R * np.sin(Theta)).astype(int)
    
    # Filtere die gültigen Indizes, die innerhalb des Bildes liegen
    valid_mask = (
        (x >= 0) & (x < radar_image.shape[1]) &
        (y >= 0) & (y < radar_image.shape[0])
    )
    
    # Setze die gültigen Koordinaten auf Schwarz (0)
    radar_image[y[valid_mask], x[valid_mask]] = 0
    # cv2.imshow("Radar_filter", radar_image)


def update_radar_image(radar_image, radar_data_theta):

    start_update = time.time()

    all_coords = np.array(radar_data_theta, dtype=np.int32)

    end_update = time.time()
    
    radar_image[all_coords[:, 1], all_coords[:, 0]] = 255
    # cv2.imshow("Radar", radar_image)
    # cv2.waitKey(0)

    print(f"............... Time update radar image: {end_update - start_update:.4f} sec")

r_100 = [
    (89.16, -45.29), (89.54, -44.52), (89.92, -43.75), (90.29, -42.98),
    (90.66, -42.20), (91.02, -41.42), (91.37, -40.64), (91.72, -39.85),
    (92.05, -39.06), (92.39, -38.27), (92.71, -37.48), (93.03, -36.68),
    (93.34, -35.88), (93.65, -35.08), (93.94, -34.27), (94.23, -33.46),
    (94.52, -32.65), (94.80, -31.84), (95.07, -31.02), (95.33, -30.21),
    (95.58, -29.39), (95.83, -28.57), (96.07, -27.74), (96.31, -26.92),
    (96.54, -26.09), (96.76, -25.26), (96.97, -24.43), (97.18, -23.59),
    (97.38, -22.76), (97.57, -21.92), (97.75, -21.08), (97.93, -20.24),
    (98.10, -19.40), (98.26, -18.56), (98.42, -17.71), (98.57, -16.87),
    (98.71, -16.02), (98.84, -15.17), (98.97, -14.32), (99.09, -13.47),
    (99.20, -12.62), (99.30, -11.77), (99.40, -10.92), (99.49, -10.06),
    (99.58, -9.21), (99.65, -8.35), (99.72, -7.50), (99.78, -6.64),
    (99.83, -5.78), (99.88, -4.93), (99.92, -4.07), (99.95, -3.21),
    (99.97, -2.35), (99.99, -1.49), (100.00, -0.64), (100.00, 0.22),
    (99.99, 1.08), (99.98, 1.94), (99.96, 2.80), (99.93, 3.66),
    (99.90, 4.51), (99.86, 5.37), (99.81, 6.23), (99.75, 7.08),
    (99.68, 7.94), (99.61, 8.80), (99.53, 9.65), (99.45, 10.51),
    (99.35, 11.36), (99.25, 12.21), (99.14, 13.06), (99.03, 13.91),
    (98.90, 14.76), (98.77, 15.61), (98.64, 16.46), (98.49, 17.31),
    (98.34, 18.15), (98.18, 18.99), (98.01, 19.84), (97.84, 20.68),
    (97.66, 21.52), (97.47, 22.35), (97.27, 23.19), (97.07, 24.03),
    (96.86, 24.86), (96.64, 25.69), (96.42, 26.52), (96.19, 27.34),
    (95.95, 28.17), (95.71, 28.99), (95.45, 29.81), (95.19, 30.63),
    (94.93, 31.45), (94.65, 32.26), (94.37, 33.07), (94.09, 33.88),
    (93.79, 34.69), (93.49, 35.49), (93.18, 36.29), (92.87, 37.09)
]
test = {50: [(44.578414409766445, -22.644314268953412),
  (44.77118981850295, -22.260740379322005),
  (44.96066485116222, -21.87552550549279),
  (45.1468255403052, -21.488698044163705),
  (45.3296581628147, -21.10028651090704),
  (45.509149240907, -20.71031953806734),
  (45.68528554312536, -20.318825872650727),
  (45.858054085315466, -19.925834374205785),
  (46.02744213158252, -19.531374012696133),
  (46.193437195230096, -19.135473866364848),
  (46.35602703968063, -18.738163119590936),
  (46.51519967937744, -18.33947106073794),
  (46.670943380668284, -17.939427079994914),
  (46.8232466626703, -17.5380606672099),
  (46.97209829811633, -17.135401409716007),
  (47.1174873141826, -16.731478990150343),
  (47.25940299329754, -16.326323184265945),
  (47.397834873931906, -15.919963858736768),
  (47.532772751369905, -15.512430968956052),
  (47.66420667846151, -15.103754556828102),
  (47.792126966355674, -14.693964748553705),
  (47.91652418521461, -14.28309175240933),
  (48.037389164908895, -13.871165856520282),
  (48.15471299569345, -13.458217426627964),
  (48.268487028864385, -13.044276903851424),
  (48.37870287739648, -12.629374802443339),
  (48.485352416561504, -12.21354170754061),
  (48.58842778452714, -11.79680827290974),
  (48.68792138293649, -11.379205218687128),
  (48.78382587746823, -10.960763329114508),
  (48.87613419837728, -10.541513450269628),
  (48.96483954101593, -10.12148648779239),
  (49.04993536633546, -9.700713404606594),
  (49.13141540136823, -9.279225218637457),
  (49.209273639690004, -8.857053000525083),
  (49.28350434186282, -8.434227871334036),
  (49.35410203585803, -8.01078100025922),
  (49.421061517459655, -7.586743602328169),
  (49.4843778506481, -7.162146936100014),
  (49.544046367963944, -6.7370223013611845),
  (49.60006267085205, -6.311401036818114),
  (49.65242262998578, -5.885314517787052),
  (49.70112238557146, -5.458794153881189),
  (49.7461583476328, -5.031871386695245),
  (49.78752719627567, -4.604577687487713),
  (49.82522588193271, -4.176944554860895),
  (49.8592516255882, -3.7490035124389447),
  (49.88960191898291, -3.320786106544054),
  (49.91627452479899, -2.892323903870966),
  (49.939267476824895, -2.463648489159988),
  (49.95857908010032, -2.0347914628686765),
  (49.9742079110412, -1.605784438842361),
  (49.98615281754457, -1.1766590419836809),
  (49.99441291907356, -0.747446905921308),
  (49.99898760672228, -0.3181796706780265),
  (49.99987654326068, 0.11111101966166023),
  (49.99707966315949, 0.5403935192842207),
  (49.99059717259494, 0.9696361829799147),
  (49.980429549433644, 1.3988073684755673),
  (49.966577543197346, 1.8278754387671279),
  (49.949042175007655, 2.256808764451839),
  (49.92782473751083, 2.6855757260598465),
  (49.902926794782395, 3.1141447163850753),
  (49.87435018221193, 3.542484142815203),
  (49.842097006367695, 3.97056242966056),
  (49.80616964484142, 4.398348020481774),
  (49.76657074607299, 4.825809380416005),
  (49.7233032291552, 5.2529149985015815),
  (49.67637028361861, 5.679633390000886),
  (49.62577536919639, 6.105933098721291),
  (49.571522215569296, 6.531782699334006),
  (49.51361482209074, 6.957150799690642),
  (49.45205745749193, 7.382006043137329),
  (49.38685465956727, 7.806317110826225),
  (49.318011234839766, 8.23005272402422),
  (49.24553225820675, 8.653181646418714),
  (49.1694230725658, 9.075672686420234),
  (49.08968928842079, 9.497494699461768),
  (49.00633678346844, 9.918616590294649),
  (48.91937170216489, 10.339007315280774),
  (48.82880045527285, 10.758635884681041),
  (48.73462971938898, 11.1774713649398),
  (48.636866436451726, 11.595482880965166),
  (48.53551781322959, 12.01263961840503),
  (48.430591320789844, 12.428910825918576),
  (48.32209469394783, 12.844265817443176),
  (48.210035930696755, 13.258673974456443),
  (48.094423291618085, 13.672104748233336),
  (47.97526529927264, 14.084527662098095),
  (47.85257073757235, 14.495912313670884),
  (47.72634865113266, 14.906228377108945),
  (47.5966083446059, 15.31544560534211),
  (47.46335938199527, 15.723533832302516),
  (47.32661158594992, 16.130462975148337),
  (47.18637503704077, 16.536203036481385),
  (47.04266007301746, 16.94072410655842),
  (46.89547728804625, 17.343996365495997),
  (46.7448375319291, 17.74599008546866),
  (46.590751909303826, 18.146675632900404),
  (46.433231778825515, 18.546023470649136)],
 200: [(178.31365763906578, -90.57725707581365),
  (179.0847592740118, -89.04296151728802),
  (179.84265940464888, -87.50210202197115),
  (180.5873021612208, -85.95479217665482),
  (181.3186326512588, -84.40114604362816),
  (182.036596963628, -82.84127815226935),
  (182.74114217250144, -81.27530349060291),
  (183.43221634126186, -79.70333749682314),
  (184.10976852633007, -78.12549605078453),
  (184.77374878092039, -76.54189546545939),
  (185.42410815872253, -74.95265247836375),
  (186.06079871750975, -73.35788424295176),
  (186.68377352267314, -71.75770831997966),
  (187.2929866506812, -70.1522426688396),
  (187.88839319246532, -68.54160563886403),
  (188.4699492567304, -66.92591596060137),
  (189.03761197319017, -65.30529273706378),
  (189.59133949572762, -63.67985543494707),
  (190.13109100547962, -62.049723875824206),
  (190.65682671384604, -60.41501822731241),
  (191.1685078654227, -58.77585899421482),
  (191.66609674085845, -57.13236700963732),
  (192.14955665963558, -55.48466342608113),
  (192.6188519827738, -53.83286970651186),
  (193.07394811545754, -52.177107615405696),
  (193.5148115095859, -50.517499209773355),
  (193.94140966624602, -48.85416683016244),
  (194.35371113810857, -47.18723309163896),
  (194.75168553174595, -45.51682087474851),
  (195.13530350987293, -43.84305331645803),
  (195.50453679350912, -42.16605380107851),
  (195.8593581640637, -40.48594595116956),
  (196.19974146534184, -38.802853618426376),
  (196.52566160547292, -37.11690087454983),
  (196.83709455876001, -35.42821200210033),
  (197.1340173674513, -33.736911485336144),
  (197.4164081434321, -32.04312400103688),
  (197.68424606983862, -30.346974409312676),
  (197.9375114025924, -28.648587744400057),
  (198.17618547185577, -26.948089205444738),
  (198.4002506834082, -25.245604147272456),
  (198.60969051994311, -23.541258071148206),
  (198.80448954228584, -21.835176615524755),
  (198.9846333905312, -20.12748554678098),
  (199.15010878510267, -18.41831074995085),
  (199.30090352773084, -16.70777821944358),
  (199.4370065023528, -14.996014049755779),
  (199.55840767593165, -13.283144426176216),
  (199.66509809919597, -11.569295615483863),
  (199.75706990729958, -9.854593956639953),
  (199.83431632040129, -8.139165851474706),
  (199.8968316441648, -6.423137755369444),
  (199.94461127017829, -4.7066361679347235),
  (199.97765167629424, -2.989787623685232),
  (199.99595042688912, -1.272718682712106),
  (199.99950617304273, 0.4444440786466409),
  (199.98831865263796, 2.161574077136883),
  (199.96238869037975, 3.878544731919659),
  (199.92171819773458, 5.595229473902269),
  (199.86631017278938, 7.3115017550685115),
  (199.79616870003062, 9.027235057807356),
  (199.71129895004333, 10.742302904239386),
  (199.61170717912958, 12.456578865540301),
  (199.4974007288477, 14.169936571260813),
  (199.36838802547078, 15.88224971864224),
  (199.22467857936567, 17.593392081927096),
  (199.06628298429197, 19.30323752166402),
  (198.8932129166208, 21.011659994006326),
  (198.70548113447444, 22.718533560003543),
  (198.50310147678556, 24.423732394885164),
  (198.28608886227718, 26.127130797336022),
  (198.05445928836295, 27.82860319876257),
  (197.80822982996773, 29.528024172549316),
  (197.54741863826908, 31.2252684433049),
  (197.27204493935906, 32.92021089609688),
  (196.982129032827, 34.61272658567486),
  (196.6776922902632, 36.302690745680934),
  (196.35875715368317, 37.98997879784707),
  (196.02534713387377, 39.674466361178595),
  (195.67748680865955, 41.3560292611231),
  (195.3152018210914, 43.034543538724165),
  (194.9385188775559, 44.7098854597592),
  (194.5474657458069, 46.381931523860665),
  (194.14207125291836, 48.05055847362012),
  (193.72236528315938, 49.715643303674305),
  (193.28837877579133, 51.377063269772705),
  (192.84014372278702, 53.03469589782577),
  (192.37769316647234, 54.68841899293334),
  (191.90106119709057, 56.33811064839238),
  (191.4102829502894, 57.983649254683534),
  (190.90539460453064, 59.62491350843578),
  (190.3864333784236, 61.26178242136844),
  (189.85343752798107, 62.894135329210066),
  (189.30644634379968, 64.52185190059335),
  (188.74550014816307, 66.14481214592554),
  (188.17064029206983, 67.76289642623368),
  (187.581909152185, 69.37598546198399),
  (186.9793501277164, 70.98396034187463),
  (186.3630076372153, 72.58670253160162),
  (185.73292711530206, 74.18409388259654)],
 300: [(267.47048645859866, -135.86588561372048),
  (268.6271389110177, -133.56444227593204),
  (269.7639891069733, -131.25315303295673),
  (270.8809532418312, -128.93218826498222),
  (271.97794897688823, -126.60171906544224),
  (273.054895445442, -124.26191722840402),
  (274.11171325875216, -121.91295523590435),
  (275.1483245118928, -119.55500624523471),
  (276.1646527894951, -117.1882440761768),
  (277.16062317138056, -114.8128431981891),
  (278.1361622380838, -112.42897871754562),
  (279.0911980762646, -110.03682636442763),
  (280.0256602840097, -107.6365624799695),
  (280.9394799760218, -105.2283640032594),
  (281.832589788698, -102.81240845829603),
  (282.7049238850956, -100.38887394090207),
  (283.5564179597852, -97.95793910559567),
  (284.3870092435914, -95.51978315242062),
  (285.19663650821946, -93.0745858137363),
  (285.98524007076907, -90.62252734096862),
  (286.75276179813403, -88.16378849132224),
  (287.4991451112877, -85.69855051445597),
  (288.2243349894534, -83.22699513912168),
  (288.9282779741607, -80.74930455976778),
  (289.6109221731863, -78.26566142310854),
  (290.27221726437887, -75.77624881466004),
  (290.912114499369, -73.28125024524367),
  (291.53056670716285, -70.78084963745843),
  (292.12752829761894, -68.27523131212277),
  (292.7029552648094, -65.76457997468705),
  (293.25680519026366, -63.24908070161777),
  (293.7890372460956, -60.72891892675434),
  (294.29961219801277, -58.20428042763956),
  (294.7884924082094, -55.67535131182474),
  (295.25564183814004, -53.14231800315049),
  (295.70102605117694, -50.60536722800422),
  (296.12461221514815, -48.064686001555316),
  (296.526369104758, -45.52046161396902),
  (296.90626710388864, -42.972881616600084),
  (297.26427820778366, -40.4221338081671),
  (297.60037602511227, -37.86840622090868),
  (297.9145357799147, -35.31188710672231),
  (298.20673431342874, -32.75276492328713),
  (298.47695008579683, -30.191228320171472),
  (298.72516317765405, -27.627466124926276),
  (298.95135529159626, -25.061667329165367),
  (299.1555097535292, -22.494021074633668),
  (299.33761151389746, -19.924716639264325),
  (299.49764714879393, -17.353943423225797),
  (299.63560486094934, -14.781890934959929),
  (299.7514744806019, -12.208748777212058),
  (299.8452474662472, -9.634706633054167),
  (299.91691690526744, -7.059954251902085),
  (299.96647751444135, -4.484681435527848),
  (299.99392564033366, -1.9090780240681589),
  (299.9992592595641, 0.6666661179699614),
  (299.9824779789569, 3.2423611157053243),
  (299.9435830355696, 5.817817097879488),
  (299.8825772966018, 8.392844210853404),
  (299.79946525918405, 10.967252632602767),
  (299.694253050046, 13.540852586711035),
  (299.566948425065, 16.113454356359078),
  (299.41756076869433, 18.68486829831045),
  (299.24610109327153, 21.25490485689122),
  (299.0525820382062, 23.82337457796336),
  (298.8370178690485, 26.390088122890642),
  (298.599424476438, 28.954856282496028),
  (298.3398193749312, 31.51748999100949),
  (298.05822170171166, 34.07780034000531),
  (297.75465221517834, 36.635598592327746),
  (297.42913329341576, 39.19069619600403),
  (297.0816889325444, 41.742904798143854),
  (296.7123447449516, 44.29203625882398),
  (296.32112795740363, 46.83790266495735),
  (295.9080674090386, 49.38031634414533),
  (295.47319354924053, 51.91908987851229),
  (295.0165384353948, 54.4540361185214),
  (294.5381357305248, 56.98496819677061),
  (294.03802070081065, 59.5116995417679),
  (293.51623021298934, 62.034043891684654),
  (292.9728027316371, 64.55181530808625),
  (292.40777831633386, 67.0648281896388),
  (291.82119861871035, 69.572897285791),
  (291.21310687937756, 72.07583771043018),
  (290.5835479247391, 74.57346495551145),
  (289.932568163687, 77.06559490465905),
  (289.2602155841805, 79.55204384673866),
  (288.5665397497085, 82.03262848940001),
  (287.8515917956359, 84.50716597258857),
  (287.11542442543407, 86.9754738820253),
  (286.35809190679595, 89.43737026265367),
  (285.5796500676354, 91.89267363205266),
  (284.7801562919716, 94.3412029938151),
  (283.9596695156995, 96.78277785089001),
  (283.1182502222446, 99.2172182188883),
  (282.25596043810475, 101.64434463935054),
  (281.3728637282775, 104.06397819297597),
  (280.4690251915746, 106.47594051281196),
  (279.544511455823, 108.88005379740243),
  (278.59939067295306, 111.27614082389482)]}

total_len = len(r_100)
for i in test:
    total_len += len(test[i])

def visualize_pair_agg(data, frame_path):
    start_time = time.time()

    global radar_data_agg
    global radar_image
    global dx, dy
    global r_100, test
    
    """
    Visualisiert ein synchronisiertes Paar aus Radar- und Kameradaten.
    """
  
    # Radar-Daten (Dummy-Visualisierung)
    radar_data = data["radar"]["data"]
    
    # Punktwolke dekodieren
    points = np.array(radar_data, dtype=np.float32).reshape(-1, 4)  # x, y, z, intensity
    new_points = points[:, :2]

    r_100 = np.array(r_100, dtype=np.float32)[:, :2]  # Sicherstellen, dass r_100 float32 ist
    new_radar_pts = [new_points, r_100]  # Liste von Arrays

    for i in test:
        new_data = np.array(test[i], dtype=np.float32)[:, :2]  # Auch test[i] in float32 umwandeln
        new_radar_pts.append(new_data)

    new_radar_pts = np.vstack(new_radar_pts)

    try:
        imu_data = data["tf"]["imu_rotation"] # DONT DELETE THIS IS NECESSARY
        # Punkte in separate Arrays zerlegen
        x_coords = new_radar_pts[:, 0]
        y_coords = new_radar_pts[:, 1]
        z_coords = points[:, 2]
        intensities = points[:, 3]
        intensities = np.append(intensities, np.ones(total_len) * 255)
    except:
        x_coords= points[:, 0]
        y_coords = points[:, 1]
        z_coords = points[:, 2]
        intensities = points[:, 3]

    x_coords_theta= points[:, 0]
    y_coords_theta = points[:, 1]
    time_radar = time.time()

    # # Radar zu Kamera transformieren
    # points_base = transform_points_radar_base(points[:, :3])
    # base_camera = transform_points_base_camera(points_base)
    # camera_points = transform_axes_to_camera(base_camera)

    # Projektion ins Bild
    undistorted_points = undistort_radar_points(new_radar_pts[:, :2], camera_matrix_center, D, K_new)

    # camera_proj = project_points(camera_points, camera_matrix_center)

    try:
        camera_proj = run_transform_with_imu(undistorted_points, data["tf"]["imu_rotation"], camera_matrix_center)
    except:
        camera_proj = run_transform(points[:, :3], camera_matrix_center)

    x_coords_cam = camera_proj[0]
    y_coords_cam = camera_proj[1]

    time_projection = time.time()

    # Kamera-Bild
    image_data = data["center_camera"]["image_data"]
    image_array = np.frombuffer(image_data, dtype=np.uint8)
    
    image = undistorted_image_center(image_array)
    
    # Konvertiere von BGR nach RGB
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    valid_mask = (x_coords_cam >= 0) & (x_coords_cam < image_rgb.shape[1]) & \
             (y_coords_cam >= 0) & (y_coords_cam < image_rgb.shape[0])

    # Beziehe nur die transformierten Radarpunkte welche im Kamerabild liegen
    x_coords_camn = x_coords_cam[valid_mask]
    y_coords_camn = y_coords_cam[valid_mask]
    
    time_image = time.time()

    r = np.sqrt(x_coords_theta**2 + y_coords_theta**2)
    theta = np.arctan2(y_coords_theta, x_coords_theta) + np.pi / 2

    theta[theta < 0] += 2 * np.pi

    theta = np.round(theta, 6).astype(np.float32)

    valid_mask_theta = (theta != 1.570796) & (theta != 4.712389)

    r = r[valid_mask_theta]
    theta = theta[valid_mask_theta]
   
    # intensities = intensities[valid_mask_theta]

    scale = radar_image.shape[0] / (MAX_RANGE * 2)
    r_scaled = r * scale
    radar_x = (r_scaled * np.cos(theta) + radar_image.shape[1] // 2).astype(int)
    radar_y = (-r_scaled * np.sin(theta) + radar_image.shape[0] // 2).astype(int)

    unique_thetas = np.unique(theta)

    theta_min, theta_max = unique_thetas[0], unique_thetas[-1]
    print(theta_max, theta_min)
    radar_data_theta = []
    clear_radar_image(radar_image, unique_thetas, MAX_RANGE * scale)

    for t, x, y in zip(theta, radar_x, radar_y):
        # if t not in radar_data_agg.keys():
        #     continue

        radar_data_theta.append((x, y))
    
    time_thorugh_radar_data = time.time()

    update_radar_image(radar_image, radar_data_theta)


    time_radar_points = time.time()

    # Die Koordinaten begrenzt auf das Bild
    y_coords_n = y_coords[valid_mask]
    x_coords_n = x_coords[valid_mask]

    # Beziehe die Winkel der Punkte
    theta_n = np.arctan2(y_coords_n, x_coords_n)
    r = np.sqrt(x_coords_n**2 + y_coords_n**2)

    colors = [point_color(r) for r in r]

    theta_n = theta_n + np.pi / 2  # Verschiebe den Winkel um 90° (π/2)
    theta_n[theta_n < 0] += 2 * np.pi

    theta_n = np.round(theta_n, 6).astype(np.float32)

    valid_mask_theta_n = (theta_n != 1.570796) & ((theta_n >= 1.1) & (theta_n <= 1.95))
    theta_n = theta_n[valid_mask_theta_n]
    colors = np.array(colors)[valid_mask_theta_n]
    x_coords_camn = x_coords_camn[valid_mask_theta_n]
    y_coords_camn = y_coords_camn[valid_mask_theta_n]

    if len(theta_n) > 0:

        # Konvertiere die Koordinaten zu Integern
        x_int = x_coords_camn.astype(int)
        y_int = y_coords_camn.astype(int)

        # Wende die Verschiebungen auf alle x- und y-Werte an (Broadcasting)
        x_offsets = x_int[:, None, None] + dx  # Shape: (N, 3, 3)
        y_offsets = y_int[:, None, None] + dy  # Shape: (N, 3, 3)

        # Erweitere colors, sodass es mit den neuen Indizes kompatibel ist
        colors_expanded = colors[:, None, None]  # Shape: (N, 1, 1)

        # Setze die Werte im Bild (alle auf einmal!)
        try:
            image_rgb[y_offsets, x_offsets] = colors_expanded
        except:
            pass

    time_radar_points_in_cam = time.time()

    scale = target_height / (MAX_RANGE * 2)

    center = (radar_image.shape[1] // 2, radar_image.shape[0] // 2)

    for r in [int(NOMINAL_RANGE / 2), NOMINAL_RANGE, 750,MAX_RANGE]:
        cv2.circle(radar_image, center, int(r * scale), (255, 255, 255), 2)  # Weiße Kreise

        # Füge Text für den Radius hinzu
        cv2.putText(radar_image, f"{r}m", 
            (radar_image.shape[1] // 2 + int(r * scale), radar_image.shape[0] // 2 - 5),  # Position des Texts
            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
        
    line_length = target_height // 2

    theta_cam_left = 1.95 + np.pi # Verschiebe den Winkel um 90° (π/2)
    theta_cam_right = 1.1 + np.pi  # Verschiebe den Winkel um 90° (π/2)

    end_point1 = (int(center[0] + line_length * np.cos(theta_cam_left)),
                int(center[1] + line_length * np.sin(theta_cam_left)))

    end_point2 = (int(center[0] + line_length * np.cos(theta_cam_right)),
                int(center[1] + line_length * np.sin(theta_cam_right)))

    cv2.line(radar_image, (radar_image.shape[1] // 2, radar_image.shape[0] // 2), end_point1, (255), 2)
    cv2.line(radar_image, (radar_image.shape[1] // 2, radar_image.shape[0] // 2), end_point2, (255), 2)

    time_draw_circles_in_radar = time.time()
 

    # Beispiel-Zielmaße (anpassbar)
    final_display_height = 1024

    # Berechne den Skalierungsfaktor für Radar- und Kamerabild
    scale_radar = final_display_height / radar_image.shape[0]

    # Neue Breiten berechnen
    radar_new_width = int(radar_image.shape[1] * scale_radar)

    # Beide Bilder auf die Zielhöhe resizen
    radar_resized = cv2.resize(radar_image, (radar_new_width, final_display_height), interpolation=cv2.INTER_AREA)
    # cam_resized   = cv2.resize(image_rgb, (cam_new_width, final_display_height), interpolation=cv2.INTER_AREA)

    radar_resized = cv2.cvtColor(radar_resized, cv2.COLOR_GRAY2BGR)
    # Bilder horizontal kombinieren
    combined_image = np.hstack((radar_resized, image_rgb))
    max_width = 1920
    max_height = 1080

    # Calculate the scaling factor while preserving aspect ratio
    scale = min(max_width / combined_image.shape[1], max_height / combined_image.shape[0])
    new_width = int(combined_image.shape[1] * scale)
    new_height = int(combined_image.shape[0] * scale)

    # Resize the combined image
    resized_combined_image = cv2.resize(combined_image, (new_width, new_height), interpolation=cv2.INTER_AREA)
    time_resize_images = time.time()
    # Display in a resizable window
    cv2.namedWindow("Combined", cv2.WINDOW_NORMAL)
    cv2.imshow("Combined", resized_combined_image)
    
    # Anzeige
    # cv2.imshow("Radar und Kamera Visualisierung", resized_combined_image)


    time_show_image = time.time()

    # print(f"Radar Processing: {time_radar - start_time:.4f} sec")
    # print(f"Projection: {time_projection - time_radar:.4f} sec")
    # print(f"Image Processing: {time_image - time_projection:.4f} sec")
    # print(f"Through Radar Data: {time_thorugh_radar_data - time_image:.4f} sec")
    # print(f"Radar points: {time_radar_points - time_thorugh_radar_data:.4f} sec")
    # print(f"points in cam: {time_radar_points_in_cam - time_radar_points:.4f} sec")
    # print(f"Draw circles in radar: {time_draw_circles_in_radar - time_radar_points_in_cam:.4f} sec")
    # print(f"Resize images: {time_resize_images - time_draw_circles_in_radar:.4f} sec")
    # print(f"Show image: {time_show_image - time_resize_images:.4f} sec")
    # print(f"--------Total Time show image: {time_show_image - start_time:.4f} sec")

    # Speichere das Bild
    # cv2.imwrite(frame_path, resized_combined_image)
   

current_dir = os.getcwd()

# file_name = "synchronized_pairs_radar_camera_v1.pkl"
file_name = "synchronized_pairs_radar_camera_tf.pkl"
SYNCHRONIZED_FILE = os.path.join(current_dir, file_name)

print(f"Pfad zur Datei: {SYNCHRONIZED_FILE}")

print(os.path.exists(SYNCHRONIZED_FILE))

# Laden der Daten aus der Pickle-Datei
with open(SYNCHRONIZED_FILE, "rb") as f:
    synchronized_pairs = pickle.load(f)
    count_save_fig = 0
    
    for data in synchronized_pairs[7500:]:
        frame_path = os.path.join(current_dir, "radar_and_camera_with_points", f"radar_cam_points_{count_save_fig}.png")
        visualize_pair_agg(data, frame_path)
        count_save_fig += 1
        
        # Warte für frame_delay Millisekunden (z.B. 30 ms für etwa 33 FPS)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # 'q' drücken, um das Video zu beenden
            break
    cv2.destroyAllWindows()
