#!/usr/bin/env python3

from fastapi import Request
from fastapi.responses import RedirectResponse
from starlette.middleware.base import BaseHTTPMiddleware

from nicegui import app, ui

from styles import load_styles

# IMPORT PAGES
import login_page
import dashboard_page

# LOAD GLOBAL CSS
load_styles()

# ============================================================
# AUTH MIDDLEWARE
# ============================================================

unrestricted_page_routes = {
    '/favicon.ico',
    '/login',
}


@app.add_middleware
class AuthMiddleware(BaseHTTPMiddleware):

    async def dispatch(
        self,
        request: Request,
        call_next
    ):

        path = request.url.path

        if (
            app.storage.user.get('authenticated')
            or path in unrestricted_page_routes
            or path.startswith('/_nicegui')
        ):

            return await call_next(request)

        return RedirectResponse(
            f'/login?redirect_to={path}'
        )


# ============================================================
# RUN
# ============================================================

ui.run(
    title='Greenhouse Robot HMI',
    storage_secret='mm_group17',
    reload=False
)