from fastapi.responses import RedirectResponse
from nicegui import ui, app

from auth import authenticate


@ui.page('/login')
def login_page(redirect_to: str = '/'):

    if app.storage.user.get('authenticated'):
        return RedirectResponse('/')

    ui.query('body').classes(add='light-theme')

    with ui.column().classes(
        'absolute-center items-center'
    ):

        with ui.card().classes(
            'glass-card light-card items-stretch'
        ):

            ui.label(
                'Digital Twin Portal'
            ).classes(
                'title text-center w-full'
            )

            ui.label(
                'Access Control'
            ).classes(
                'text-center text-gray-500'
            )
            ui.separator()

            # ------------------------------------------------
            # LOGIN SECTION
            # ------------------------------------------------

            ui.label(
                'Login'
            ).classes(
                'text-lg'
            )

            username = ui.input(
                'Username'
            ).props(
                'outlined autofocus'
            ).classes(
                'w-full'
            )

            password = ui.input(
                'Password',
                password=True,
                password_toggle_button=True
            ).props(
                'outlined'
            ).classes(
                'w-full'
            )

            def try_login():

                if authenticate(
                    username.value,
                    password.value
                ):

                    app.storage.user.update(
                        username=username.value,
                        authenticated=True
                    )

                    ui.notify(
                        'Login Successful',
                        color='positive'
                    )

                    ui.navigate.to('/')

                else:

                    ui.notify(
                        'Login Failed',
                        color='negative'
                    )

            ui.button(
                'LOGIN',
                on_click=try_login
            ).classes(
                'login-btn w-full h-12'
            )