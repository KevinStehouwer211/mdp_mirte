from nicegui import ui

def load_styles():

    ui.add_head_html("""
    <link rel="preconnect" href="https://fonts.googleapis.com">
    <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>

    <link href="https://fonts.googleapis.com/css2?family=Montserrat:wght@300;400;600;700&display=swap" rel="stylesheet">

    <style>

    /* =========================================================
    GENERAL BODY STYLING
    ========================================================= */

    body {

        font-family: 'Montserrat', sans-serif;

        transition:
            background 0.3s ease,
            color 0.3s ease;

        overflow: hidden;
    }


    /* =========================================================
    LIGHT MODE
    ========================================================= */

    .light-theme {

        background:
            linear-gradient(
                135deg,
                white 0%,
                #ebfadc 100%
            );

        color: black;
    }

    /* =========================================================
    GLASS CARD
    ========================================================= */

    .glass-card {

        width: 430px;

        padding: 32px;

        border-radius: 5px;

        backdrop-filter: blur(16px);

        transition:
            transform 0.25s ease,
            box-shadow 0.25s ease,
            background 0.3s ease;

        position: relative;
    }

    /* =========================================================
    SIDE CARD
    ========================================================= */

    .side-card {
        width: 20%;
        height: 100vh;           /* fill full page height */
        padding: 32px;
        border-radius: 0 5px 5px 0;   /* flat on left edge, rounded on right */
        backdrop-filter: blur(16px);
        transition:
            transform 0.25s ease,
            box-shadow 0.25s ease,
            background 0.3s ease;
        position: fixed !important;  /* fixed so it stays during scroll */
        top: 0;
        left: 0;
        overflow-y: auto;        /* scroll internally if content overflows */
        z-index: 100;
    }   



    /* =========================================================
    LIGHT CARD
    ========================================================= */

    .light-card {

        background:
            rgba(255,255,255,0.3) !important;

        border:
            1px solid white;

        box-shadow:
            0 8px 32px rgba(15,23,42,0.10);
    }

    /* =========================================================
    CARD HOVER EFFECT
    ========================================================= */

    .glass-card:hover {

        transform: translateY(-4px);

        box-shadow:
            0 12px 36px rgba(0,0,0,0.22);
    }

    /* =========================================================
    TITLES
    ========================================================= */

    .title {

        font-size: 34px;

        font-weight: 500;

        letter-spacing: 0.4px;

        color: black;
    }

    /* =========================================================
    SUBTITLE
    ========================================================= */

    .subtitle {

        font-size: 15px;

        color: #94a3b8;

        margin-top: -4px;
    }

    /* =========================================================
    INPUT FIELDS
    ========================================================= */

    .q-field__control {

        border-radius: 14px !important;
                 
    }

    input::placeholder {
        color: #94a3b8 !important;
        opacity: 1;
    }
                 
    /* =========================================================
    PRIMARY BUTTON
    ========================================================= */

    .login-btn {

        background:
            green !important;

        color: white !important;

        font-weight: 600;

        letter-spacing: 0.4px;

        border-radius: 14px;

        transition:
            transform 0.2s ease,
            opacity 0.2s ease,
            box-shadow 0.2s ease;
    }

    /* =========================================================
    BUTTON HOVER
    ========================================================= */

    .login-btn:hover {

        transform: scale(1.02);

        opacity: 0.96;
    }

    /* =========================================================
    SECONDARY BUTTON
    ========================================================= */

    .secondary-btn {

        background:
            linear-gradient(
                135deg,
                #0ea5e9 0%,
                #38bdf8 100%
            ) !important;

        color: white !important;

        font-weight: 600;

        border-radius: 14px;

        box-shadow:
            0 4px 18px rgba(14,165,233,0.28);

        transition:
            transform 0.2s ease,
            opacity 0.2s ease;
    }

    .secondary-btn:hover {

        transform: scale(1.02);

        opacity: 0.96;
    }

    /* =========================================================
    TOGGLE / SWITCH ACCENTS
    ========================================================= */


    .q-toggle__track {

        opacity: 0.35 !important;
    }

    /* =========================================================
    RESPONSIVE DESIGN
    ========================================================= */

    @media (max-width: 768px) {

        .glass-card {

            width: 92vw;

            padding: 24px;
        }

        .title {

            font-size: 28px;
        }
    }
     
    .main-container {
        width: 100vw;
        height: 100vh;
        display: flex;
        justify-content: center;
        align-items: center;
    }

    .map-panel {
        position: absolute;
        width: 76vw;
        height: 60vh;
        background: rgba(255,255,255,0.5);
        /*border-radius: 24px;*/
        position: fixed !important;  /* fixed so it stays during scroll */
        top: 2vh;
        left: 22vw;
        overflow: hidden;
        /*box-shadow: 0 12px 40px rgba(0,0,0,0.12);*/
    }

    .greenhouse-border {
        position: absolute;
        left: 0px;
        top: 0px;
        width: 76vw;
        height: 60vh;
        border: 3px solid black;
        box-sizing: border-box;
    }

    .home-station {
        position: absolute;
        right: 60px;
        top: 40px;
        width: 140px;
        height: 120px;
        border: 5px solid black;
        background: white;
        display: flex;
        align-items: center;
        justify-content: center;
        font-size: 36px;
        font-weight: 500;
    }

    .plant-bed {
        position: absolute;
        width: 3vw;
        height: 15vh;
        background: #92745B;
        /*border-radius: 14px;*/
        border: 3px dotted black;
    }

    .entity {
        position: absolute;
        border-radius: 50%;
        cursor: pointer;
        transition: transform 0.2s ease;
    }
    .entity:hover { transform: scale(1.15); }

    .plant-dot  { width: 14px; height: 14px; background: #39ff14; }
    .bug-dot    { width: 14px; height: 14px; background: #ff3b3b; }
    .sensor-dot { width: 14px; height: 14px; background: #ffd43b; }

    /* Robot uses CSS custom properties so JS can update
       position without replacing the whole element */
    
    .robot {

    position: absolute;

    width: 34px;
    height: 34px;

    background: #2563eb;

    clip-path: polygon(
        50% 0%,
        100% 100%,
        50% 75%,
        0% 100%
    );

    box-shadow:
        0 0 20px rgba(37,99,235,0.45);

    transition:
        left 0.05s linear,
        top 0.05s linear,
        transform 0.1s linear;

    z-index: 50;
    }

    .tooltip-js {
        position: absolute;
        top: 20px;
        left: 50%;
        transform: translateY(-20%);
        min-width: 180px;
        background: rgba(255,255,255,0.7);
        backdrop-filter: blur(10px);
        border-radius: 16px;
        padding: 12px;
        display: none;
        z-index: 100;
        box-shadow: 0 8px 24px rgba(0,0,0,0.15);
        /* prevent tooltip from closing while hovering it */
        pointer-events: auto;
    }
    .entity:hover .tooltip-js { display: block; }

    .tooltip-btn {
        width: 100%;
        border: none;
        padding: 10px;
        margin-top: 8px;
        border-radius: 10px;
        background: linear-gradient(135deg, #22c55e, #15803d);
        color: white;
        cursor: pointer;
    }

    .coord-system {
        position: absolute;
        left: 15px;
        bottom: 10px;
        font-size: 20px;
        font-weight: bold;
    }
    
    .alert-panel {
        width: 37vw;
        height: 30vh;
        background: rgba(255,255,255,0.5);
        /*border-radius: 24px;*/
        position: fixed !important;  /* fixed so it stays during scroll */
        top: 66vh;
        left: 22vw;
        overflow: hidden;
        /*box-shadow: 0 12px 40px rgba(0,0,0,0.12);*/
    }
    
    .warning-panel {
        width: 37vw;
        height: 30vh;
        background: rgba(255,255,255,0.5);
        /*border-radius: 24px;*/
        position: fixed !important;  /* fixed so it stays during scroll */
        top: 66vh;
        left: 61vw;
        overflow: hidden;
        /*box-shadow: 0 12px 40px rgba(0,0,0,0.12);*/
    }
    
    /* =========================================================
    BLOOM ANIMATION FOR PLANTS
    ========================================================= */
    
    @keyframes bloomBlink {

    0% {
        transform: scale(1);
        /*box-shadow: 0 0 0px rgba(57,255,20,0.0);*/
        opacity: 1;
    }

    50% {
        transform: scale(1.1);
        /*box-shadow: 0 0 18px rgba(57,255,20,0.9);*/
        opacity: 0.75;
    }

    100% {
        transform: scale(1);
        /*box-shadow: 0 0 0px rgba(57,255,20,0.0);*/
        opacity: 1;
    }
    }

    .blooming {

        animation:
            bloomBlink 1.2s infinite ease-in-out;
    }        

    </style>
    """, shared=True)