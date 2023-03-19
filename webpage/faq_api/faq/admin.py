from django.contrib import admin
from .models import QuestionTable, AnswerTable

# Register your models here.
admin.site.register(QuestionTable)
admin.site.register(AnswerTable)
