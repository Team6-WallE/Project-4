from django.shortcuts import render
from rest_framework import viewsets
from .models import QuestionTable, AnswerTable
from .serializers import QuestionSerializer, AnswerSerializer

# Create your views here.
class QuestionView(viewsets.ModelViewSet):
    queryset = QuestionTable.objects.all()
    serializer_class = QuestionSerializer

class AnswerView(viewsets.ModelViewSet):
    queryset = AnswerTable.objects.all()
    serializer_class = AnswerSerializer