from django.urls import path, include
from . import views
from rest_framework import routers

router = routers.DefaultRouter()
router.register('question', views.QuestionView)
router.register('answer', views.AnswerView)

urlpatterns = [
    path('', include(router.urls))
]
