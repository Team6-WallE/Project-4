from django.db import models
import rest_framework


# Create your models here.
class QuestionTable(models.Model):
    question_id = models.AutoField(primary_key=True)
    questions = models.TextField()

    def __str__(self):
        return self.questions


class AnswerTable(models.Model):
    question_id = models.ForeignKey(QuestionTable, null=True, on_delete=models.SET_NULL)
    answers = models.TextField()

    def __str__(self):
        return self.question_id